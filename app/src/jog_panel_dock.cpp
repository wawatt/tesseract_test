#include "jog_panel_dock.h"
#include "simulation_controller.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QTabWidget>
#include <QGroupBox>
#include <QScrollArea>
#include <cmath>

namespace sim_app {

static void quatToRpyDeg(double qx, double qy, double qz, double qw, double& roll_deg, double& pitch_deg, double& yaw_deg) {
    const double rad2deg = 180.0 / 3.14159265358979323846;
    double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    roll_deg = std::atan2(sinr_cosp, cosr_cosp) * rad2deg;

    double sinp = 2.0 * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1.0)
        pitch_deg = std::copysign(90.0, sinp);
    else
        pitch_deg = std::asin(sinp) * rad2deg;

    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    yaw_deg = std::atan2(siny_cosp, cosy_cosp) * rad2deg;
}

static QWidget* makeScrollable(QWidget* inner) {
    QScrollArea* sa = new QScrollArea();
    sa->setWidgetResizable(true);
    sa->setFrameShape(QFrame::NoFrame);
    sa->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    sa->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    sa->setWidget(inner);
    return sa;
}

JogPanelDock::JogPanelDock(SimulationController* controller, QWidget* parent)
    : QDockWidget("示教与微调控制 (Teach & Jog)", parent), controller_(controller) {
    setObjectName("JogPanelDock");
    setupUi();

    connect(controller_, &SimulationController::sigJointsUpdated, this, &JogPanelDock::onJointsUpdated);
    connect(controller_, &SimulationController::sigRobotLoaded, this, &JogPanelDock::onRobotLoaded);
    connect(controller_, &SimulationController::sigCartesianJogFeedback, this, &JogPanelDock::onCartesianJogFeedback);

    if (controller_->isRobotLoaded()) {
        onRobotLoaded(true, QString::fromStdString(controller_->getRobotName()));
    }
}

void JogPanelDock::setupUi() {
    QWidget* container = new QWidget(this);
    QVBoxLayout* mainLayout = new QVBoxLayout(container);
    mainLayout->setContentsMargins(6, 6, 6, 6);

    QTabWidget* tabWidget = new QTabWidget(container);
    tabWidget->addTab(makeScrollable(createJointJogTab()), "关节单轴示教 (Joint)");
    tabWidget->addTab(makeScrollable(createCartesianJogTab()), "笛卡尔末端示教 (Cartesian)");

    mainLayout->addWidget(tabWidget);
    setWidget(container);
}

QWidget* JogPanelDock::createJointJogTab() {
    QWidget* tab = new QWidget();
    QVBoxLayout* tabLayout = new QVBoxLayout(tab);
    tabLayout->setSpacing(8);

    QGroupBox* group = new QGroupBox("各轴关节角度 (Degrees)", tab);
    QVBoxLayout* groupLayout = new QVBoxLayout(group);
    groupLayout->setSpacing(6);

    joint_rows_.clear();
    for (int i = 0; i < 6; ++i) {
        QWidget* rowWidget = new QWidget(group);
        QHBoxLayout* rowLayout = new QHBoxLayout(rowWidget);
        rowLayout->setContentsMargins(0, 2, 0, 2);

        JointControlRow row;
        row.label = new QLabel(QString(" J%1 ").arg(i + 1), rowWidget);
        row.label->setFixedWidth(36);
        row.label->setAlignment(Qt::AlignCenter);
        row.label->setStyleSheet("background: #242934; border: 1px solid #363d4c; border-radius: 3px; font-weight: bold; color: #40b0ff;");

        row.slider = new QSlider(Qt::Horizontal, rowWidget);
        row.spinbox = new QDoubleSpinBox(rowWidget);
        row.spinbox->setDecimals(2);
        row.spinbox->setFixedWidth(92);
        row.spinbox->setSingleStep(1.0);
        row.spinbox->setSuffix("°");

        rowLayout->addWidget(row.label);
        rowLayout->addWidget(row.slider, 1);
        rowLayout->addWidget(row.spinbox);

        int joint_idx = i;
        connect(row.slider, &QSlider::valueChanged, this, [this, joint_idx](int val) {
            onSliderValueChanged(joint_idx, val);
        });
        connect(row.spinbox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this, joint_idx](double val) {
            onSpinBoxValueChanged(joint_idx, val);
        });

        joint_rows_.push_back(row);
        groupLayout->addWidget(rowWidget);
    }

    tabLayout->addWidget(group);

    // 快捷预设按钮
    QHBoxLayout* btnLayout = new QHBoxLayout();
    QPushButton* btnZero = new QPushButton("零位 (Zero)", tab);
    QPushButton* btnHome = new QPushButton("归位 (Home)", tab);
    btnHome->setObjectName("btnSuccess");
    QPushButton* btnReady = new QPushButton("就绪位 (Ready)", tab);
    btnReady->setObjectName("btnPrimary");

    connect(btnZero, &QPushButton::clicked, this, [this]() {
        controller_->setJoints({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    });
    connect(btnHome, &QPushButton::clicked, this, [this]() {
        controller_->moveToWaypoint("Home");
    });
    connect(btnReady, &QPushButton::clicked, this, [this]() {
        controller_->moveToWaypoint("Ready");
    });

    btnLayout->addWidget(btnZero);
    btnLayout->addWidget(btnHome);
    btnLayout->addWidget(btnReady);
    tabLayout->addLayout(btnLayout);

    tabLayout->addStretch(1);
    return tab;
}

QWidget* JogPanelDock::createCartesianJogTab() {
    QWidget* tab = new QWidget();
    QVBoxLayout* tabLayout = new QVBoxLayout(tab);
    tabLayout->setSpacing(8);

    // 1. TCP 位姿数字仪表板
    QGroupBox* displayGroup = new QGroupBox("末端工具位姿仪表 (TCP Readout)", tab);
    QGridLayout* displayGrid = new QGridLayout(displayGroup);
    displayGrid->setSpacing(6);

    auto createReadoutCell = [](const QString& axis, const QString& color) -> std::pair<QFrame*, QLabel*> {
        QFrame* frame = new QFrame();
        frame->setObjectName("digitalReadoutCard");
        QHBoxLayout* hl = new QHBoxLayout(frame);
        hl->setContentsMargins(6, 4, 6, 4);
        hl->setSpacing(6);

        QLabel* lblTag = new QLabel(axis, frame);
        lblTag->setObjectName("readoutAxisLabel");
        lblTag->setStyleSheet(QString("background: %1; color: #ffffff; font-weight: bold; border-radius: 3px; padding: 2px 5px;").arg(color));

        QLabel* lblVal = new QLabel("0.000", frame);
        lblVal->setObjectName("readoutValue");
        lblVal->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

        hl->addWidget(lblTag);
        hl->addWidget(lblVal, 1);
        return {frame, lblVal};
    };

    auto [cardX, lblX] = createReadoutCell("X", "#ff4d4f");
    auto [cardY, lblY] = createReadoutCell("Y", "#52c41a");
    auto [cardZ, lblZ] = createReadoutCell("Z", "#1890ff");
    auto [cardRx, lblRx] = createReadoutCell("Rx", "#faad14");
    auto [cardRy, lblRy] = createReadoutCell("Ry", "#faad14");
    auto [cardRz, lblRz] = createReadoutCell("Rz", "#faad14");

    label_val_x_ = lblX;
    label_val_y_ = lblY;
    label_val_z_ = lblZ;
    label_val_rx_ = lblRx;
    label_val_ry_ = lblRy;
    label_val_rz_ = lblRz;

    displayGrid->addWidget(cardX, 0, 0);
    displayGrid->addWidget(cardY, 0, 1);
    displayGrid->addWidget(cardZ, 0, 2);
    displayGrid->addWidget(cardRx, 1, 0);
    displayGrid->addWidget(cardRy, 1, 1);
    displayGrid->addWidget(cardRz, 1, 2);

    tabLayout->addWidget(displayGroup);

    // 2. 步长设置
    QGroupBox* stepGroup = new QGroupBox("步进微调参数 (Jog Steps)", tab);
    QHBoxLayout* stepLayout = new QHBoxLayout(stepGroup);

    stepLayout->addWidget(new QLabel("平移:"));
    combo_trans_step_ = new QComboBox(stepGroup);
    combo_trans_step_->addItem("1 mm", 0.001);
    combo_trans_step_->addItem("5 mm", 0.005);
    combo_trans_step_->addItem("10 mm", 0.010);
    combo_trans_step_->addItem("50 mm", 0.050);
    combo_trans_step_->addItem("100 mm", 0.100);
    combo_trans_step_->setCurrentIndex(2); // 默认 10mm
    stepLayout->addWidget(combo_trans_step_);

    stepLayout->addWidget(new QLabel("旋转:"));
    combo_rot_step_ = new QComboBox(stepGroup);
    combo_rot_step_->addItem("1°", 1.0 * 3.14159265 / 180.0);
    combo_rot_step_->addItem("5°", 5.0 * 3.14159265 / 180.0);
    combo_rot_step_->addItem("10°", 10.0 * 3.14159265 / 180.0);
    combo_rot_step_->addItem("45°", 45.0 * 3.14159265 / 180.0);
    combo_rot_step_->setCurrentIndex(1); // 默认 5°
    stepLayout->addWidget(combo_rot_step_);

    tabLayout->addWidget(stepGroup);

    // 3. 笛卡尔平移点动网格
    QGroupBox* jogTransGroup = new QGroupBox("末端平移点动 (Position XYZ)", tab);
    QGridLayout* transGrid = new QGridLayout(jogTransGroup);
    transGrid->setSpacing(6);

    auto addJogBtnPair = [this](QGridLayout* gl, const QString& decText, const QString& incText, const QString& objName, int axisIdx, int row) {
        QPushButton* btnDec = new QPushButton(decText);
        QPushButton* btnInc = new QPushButton(incText);
        btnDec->setObjectName(objName);
        btnInc->setObjectName(objName);
        btnDec->setMinimumHeight(34);
        btnInc->setMinimumHeight(34);

        connect(btnDec, &QPushButton::clicked, this, [this, axisIdx]() {
            onCartesianStepClicked(axisIdx, false);
        });
        connect(btnInc, &QPushButton::clicked, this, [this, axisIdx]() {
            onCartesianStepClicked(axisIdx, true);
        });

        gl->addWidget(btnDec, row, 0);
        gl->addWidget(btnInc, row, 1);
    };

    addJogBtnPair(transGrid, "◀ -X (后退)", "+X (前进) ▶", "btnJogX", 0, 0);
    addJogBtnPair(transGrid, "◀ -Y (向左)", "+Y (向右) ▶", "btnJogY", 1, 1);
    addJogBtnPair(transGrid, "▼ -Z (下降)", "+Z (上升) ▲", "btnJogZ", 2, 2);
    tabLayout->addWidget(jogTransGroup);

    // 4. 笛卡尔旋转点动网格
    QGroupBox* jogRotGroup = new QGroupBox("末端姿态点动 (Rotation RPY)", tab);
    QGridLayout* rotGrid = new QGridLayout(jogRotGroup);
    rotGrid->setSpacing(6);

    addJogBtnPair(rotGrid, "⟲ -Rx (负滚)", "+Rx (正滚) ⟳", "btnJogRot", 3, 0);
    addJogBtnPair(rotGrid, "⟲ -Ry (俯角)", "+Ry (仰角) ⟳", "btnJogRot", 4, 1);
    addJogBtnPair(rotGrid, "⟲ -Rz (左偏)", "+Rz (右偏) ⟳", "btnJogRot", 5, 2);
    tabLayout->addWidget(jogRotGroup);

    // 5. 碰撞拦截选项与实时交互诊断横幅
    chk_stop_on_collision_ = new QCheckBox("碰撞安全拦截保护 (Stop on Collision)", tab);
    chk_stop_on_collision_->setChecked(true);
    chk_stop_on_collision_->setStyleSheet("color: #40b0ff; font-weight: bold; font-size: 12px; margin-top: 4px;");
    tabLayout->addWidget(chk_stop_on_collision_);

    label_jog_feedback_ = new QLabel("就绪 - 点击上方按钮进行末端点动 (自动检测逆解与碰撞)", tab);
    label_jog_feedback_->setStyleSheet("background: #181b22; border: 1px solid #2d3340; border-radius: 4px; padding: 6px 10px; color: #8c93a4; font-size: 12px; font-weight: 500;");
    label_jog_feedback_->setWordWrap(true);
    tabLayout->addWidget(label_jog_feedback_);

    tabLayout->addStretch(1);
    return tab;
}

void JogPanelDock::onRobotLoaded(bool success, const QString& robot_name) {
    (void)robot_name;
    if (!success) return;

    const auto& lowers = controller_->getJointLowerLimits();
    const auto& uppers = controller_->getJointUpperLimits();
    const double rad2deg = 180.0 / 3.14159265358979323846;

    updating_ui_ = true;
    for (size_t i = 0; i < joint_rows_.size() && i < lowers.size() && i < uppers.size(); ++i) {
        double low_deg = lowers[i] * rad2deg;
        double up_deg = uppers[i] * rad2deg;
        joint_rows_[i].lower_deg = low_deg;
        joint_rows_[i].upper_deg = up_deg;

        joint_rows_[i].slider->setRange(static_cast<int>(low_deg * 10.0), static_cast<int>(up_deg * 10.0));
        joint_rows_[i].spinbox->setRange(low_deg, up_deg);
    }
    updating_ui_ = false;

    onJointsUpdated(controller_->getCurrentJoints(), controller_->getCurrentTcpPose());
}

void JogPanelDock::onJointsUpdated(const std::vector<double>& joints, const std::vector<double>& tcp_pose) {
    if (updating_ui_) return;
    updating_ui_ = true;

    const double rad2deg = 180.0 / 3.14159265358979323846;
    for (size_t i = 0; i < joint_rows_.size() && i < joints.size(); ++i) {
        double deg = joints[i] * rad2deg;
        joint_rows_[i].spinbox->setValue(deg);
        joint_rows_[i].slider->setValue(static_cast<int>(deg * 10.0));
    }

    if (tcp_pose.size() >= 7 && label_val_x_) {
        label_val_x_->setText(QString("%1 m").arg(tcp_pose[0], 6, 'f', 3));
        label_val_y_->setText(QString("%1 m").arg(tcp_pose[1], 6, 'f', 3));
        label_val_z_->setText(QString("%1 m").arg(tcp_pose[2], 6, 'f', 3));

        double r_deg, p_deg, y_deg;
        quatToRpyDeg(tcp_pose[3], tcp_pose[4], tcp_pose[5], tcp_pose[6], r_deg, p_deg, y_deg);
        label_val_rx_->setText(QString("%1°").arg(r_deg, 5, 'f', 1));
        label_val_ry_->setText(QString("%1°").arg(p_deg, 5, 'f', 1));
        label_val_rz_->setText(QString("%1°").arg(y_deg, 5, 'f', 1));
    }

    updating_ui_ = false;
}

void JogPanelDock::onSliderValueChanged(int joint_idx, int int_val) {
    if (updating_ui_) return;
    double deg = static_cast<double>(int_val) / 10.0;
    double rad = deg * (3.14159265358979323846 / 180.0);

    updating_ui_ = true;
    joint_rows_[joint_idx].spinbox->setValue(deg);
    updating_ui_ = false;

    auto joints = controller_->getCurrentJoints();
    if (joint_idx < static_cast<int>(joints.size())) {
        joints[joint_idx] = rad;
        controller_->setJoints(joints);
    }
}

void JogPanelDock::onSpinBoxValueChanged(int joint_idx, double val_deg) {
    if (updating_ui_) return;
    double rad = val_deg * (3.14159265358979323846 / 180.0);

    updating_ui_ = true;
    joint_rows_[joint_idx].slider->setValue(static_cast<int>(val_deg * 10.0));
    updating_ui_ = false;

    auto joints = controller_->getCurrentJoints();
    if (joint_idx < static_cast<int>(joints.size())) {
        joints[joint_idx] = rad;
        controller_->setJoints(joints);
    }
}

void JogPanelDock::onCartesianStepClicked(int axis, bool positive) {
    double step = 0.0;
    if (axis >= 0 && axis <= 2) {
        step = combo_trans_step_->currentData().toDouble();
    } else {
        step = combo_rot_step_->currentData().toDouble();
    }
    if (!positive) step = -step;

    bool stop_on_collision = (chk_stop_on_collision_ ? chk_stop_on_collision_->isChecked() : true);
    controller_->jogCartesian(axis, step, stop_on_collision);
}

void JogPanelDock::onCartesianJogFeedback(int status_code, const QString& message) {
    if (!label_jog_feedback_) return;
    label_jog_feedback_->setText(message);
    if (status_code == 0) { // 步进成功，无碰撞
        label_jog_feedback_->setStyleSheet("background: #0d2818; border: 1px solid #169651; border-radius: 4px; padding: 6px 10px; color: #00e676; font-size: 12px; font-weight: 500;");
    } else if (status_code == 1) { // 逆解求解失败 (非碰撞)
        label_jog_feedback_->setStyleSheet("background: #332800; border: 1px solid #faad14; border-radius: 4px; padding: 6px 10px; color: #ffd666; font-size: 12px; font-weight: 500;");
    } else if (status_code == 2) { // 发生碰撞干涉
        label_jog_feedback_->setStyleSheet("background: #331111; border: 1px solid #ff4d4f; border-radius: 4px; padding: 6px 10px; color: #ff7875; font-size: 12px; font-weight: bold;");
    }
}

} // namespace sim_app
