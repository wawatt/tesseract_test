#include "planning_dock.h"
#include "simulation_controller.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QScrollArea>
#include <QInputDialog>
#include <QMessageBox>

namespace sim_app {

PlanningDock::PlanningDock(SimulationController* controller, QWidget* parent)
    : QDockWidget("动作规划与点位库 (Motion Planner)", parent), controller_(controller) {
    setObjectName("PlanningDock");
    setupUi();

    connect(controller_, &SimulationController::sigWaypointsChanged, this, &PlanningDock::onWaypointsChanged);
    connect(controller_, &SimulationController::sigTrajectoryGenerated, this, &PlanningDock::onTrajectoryGenerated);

    onWaypointsChanged();
}

void PlanningDock::setupUi() {
    QScrollArea* scrollArea = new QScrollArea(this);
    scrollArea->setWidgetResizable(true);
    scrollArea->setFrameShape(QFrame::NoFrame);
    scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

    QWidget* container = new QWidget(scrollArea);
    QVBoxLayout* mainLayout = new QVBoxLayout(container);
    mainLayout->setContentsMargins(6, 6, 6, 6);
    mainLayout->setSpacing(8);

    mainLayout->addWidget(createWaypointsSection());
    mainLayout->addWidget(createPlanningSection());
    mainLayout->addStretch(1);

    scrollArea->setWidget(container);
    setWidget(scrollArea);
}

QWidget* PlanningDock::createWaypointsSection() {
    QGroupBox* group = new QGroupBox("示教点位库 (Waypoints)", this);
    QVBoxLayout* layout = new QVBoxLayout(group);
    layout->setSpacing(6);

    list_waypoints_ = new QListWidget(group);
    list_waypoints_->setMinimumHeight(65);
    list_waypoints_->setMaximumHeight(110);
    layout->addWidget(list_waypoints_);

    QHBoxLayout* btnLayout = new QHBoxLayout();
    QPushButton* btnSave = new QPushButton("➕ 记录当前位姿", group);
    btnSave->setObjectName("btnSuccess");
    QPushButton* btnMove = new QPushButton("📍 跳转至该点", group);
    QPushButton* btnDel = new QPushButton("🗑️ 删除点位", group);
    btnDel->setObjectName("btnDanger");

    connect(btnSave, &QPushButton::clicked, this, &PlanningDock::onSaveWaypointClicked);
    connect(btnMove, &QPushButton::clicked, this, &PlanningDock::onMoveToWaypointClicked);
    connect(btnDel, &QPushButton::clicked, this, &PlanningDock::onDeleteWaypointClicked);

    btnLayout->addWidget(btnSave);
    btnLayout->addWidget(btnMove);
    btnLayout->addWidget(btnDel);
    layout->addLayout(btnLayout);

    return group;
}

QWidget* PlanningDock::createPlanningSection() {
    QGroupBox* group = new QGroupBox("运动轨迹生成 (Trajectory Planning)", this);
    QVBoxLayout* layout = new QVBoxLayout(group);
    layout->setSpacing(8);

    QFormLayout* form = new QFormLayout();

    combo_motion_type_ = new QComboBox(group);
    combo_motion_type_->addItem("[PTP] 自由空间关节避障规划 (Freespace)");
    combo_motion_type_->addItem("[LIN] 笛卡尔空间直线插补 (Linear)");
    combo_motion_type_->addItem("[CIRC] 笛卡尔空间圆弧插补 (Circular)");
    connect(combo_motion_type_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &PlanningDock::onMotionTypeChanged);
    form->addRow("运动指令类型:", combo_motion_type_);

    combo_start_wp_ = new QComboBox(group);
    combo_target_wp_ = new QComboBox(group);
    form->addRow("起始点位 (Start):", combo_start_wp_);
    form->addRow("目标点位 (Target):", combo_target_wp_);

    widget_aux_wp_ = new QWidget(group);
    QHBoxLayout* auxLayout = new QHBoxLayout(widget_aux_wp_);
    auxLayout->setContentsMargins(0, 0, 0, 0);
    combo_aux_wp_ = new QComboBox(widget_aux_wp_);
    auxLayout->addWidget(combo_aux_wp_);
    widget_aux_wp_->setVisible(false); // 默认 PTP 不显示圆弧中间点
    form->addRow("圆弧中间点 (Aux):", widget_aux_wp_);

    // 动力学与避障参数
    spin_vel_scale_ = new QDoubleSpinBox(group);
    spin_vel_scale_->setRange(0.05, 1.0); spin_vel_scale_->setValue(1.0); spin_vel_scale_->setSingleStep(0.05);
    spin_vel_scale_->setSuffix(" x");
    form->addRow("速度缩放比 (Vel Scale):", spin_vel_scale_);

    spin_acc_scale_ = new QDoubleSpinBox(group);
    spin_acc_scale_->setRange(0.05, 1.0); spin_acc_scale_->setValue(1.0); spin_acc_scale_->setSingleStep(0.05);
    spin_acc_scale_->setSuffix(" x");
    form->addRow("加速度缩放比 (Acc Scale):", spin_acc_scale_);

    spin_safety_margin_ = new QDoubleSpinBox(group);
    spin_safety_margin_->setRange(0.005, 0.2); spin_safety_margin_->setValue(0.025); spin_safety_margin_->setSingleStep(0.005);
    spin_safety_margin_->setSuffix(" m");
    form->addRow("安全裕度 (Margin m):", spin_safety_margin_);

    spin_timeout_ = new QDoubleSpinBox(group);
    spin_timeout_->setRange(1.0, 30.0); spin_timeout_->setValue(10.0); spin_timeout_->setSingleStep(1.0);
    spin_timeout_->setSuffix(" s");
    form->addRow("规划超时 (Timeout s):", spin_timeout_);

    layout->addLayout(form);

    btn_plan_ = new QPushButton("⚡ 开始执行规划 (Plan Trajectory)", group);
    btn_plan_->setObjectName("btnPrimary");
    btn_plan_->setMinimumHeight(42);
    btn_plan_->setStyleSheet("font-size: 14px; font-weight: bold; border-radius: 5px;");
    connect(btn_plan_, &QPushButton::clicked, this, &PlanningDock::onStartPlanClicked);
    layout->addWidget(btn_plan_);

    label_result_banner_ = new QLabel("就绪 - 请选择目标点位并规划", group);
    label_result_banner_->setStyleSheet("background: #191c23; border: 1px solid #2f3542; padding: 8px 10px; border-radius: 5px; color: #a4abbc; font-size: 12px;");
    label_result_banner_->setWordWrap(true);
    layout->addWidget(label_result_banner_);

    return group;
}

void PlanningDock::onWaypointsChanged() {
    list_waypoints_->clear();
    const auto& wps = controller_->getWaypoints();
    for (const auto& pair : wps) {
        list_waypoints_->addItem(QString::fromStdString(pair.first));
    }
    refreshWaypointCombos();
}

void PlanningDock::refreshWaypointCombos() {
    QString curStart = combo_start_wp_->currentText();
    QString curTarget = combo_target_wp_->currentText();
    QString curAux = combo_aux_wp_->currentText();

    combo_start_wp_->clear();
    combo_target_wp_->clear();
    combo_aux_wp_->clear();

    combo_start_wp_->addItem("<当前机械臂位姿 (Current)>");

    const auto& wps = controller_->getWaypoints();
    for (const auto& pair : wps) {
        QString name = QString::fromStdString(pair.first);
        combo_start_wp_->addItem(name);
        combo_target_wp_->addItem(name);
        combo_aux_wp_->addItem(name);
    }

    if (!curStart.isEmpty()) combo_start_wp_->setCurrentText(curStart);
    if (!curTarget.isEmpty()) combo_target_wp_->setCurrentText(curTarget);
    if (!curAux.isEmpty()) combo_aux_wp_->setCurrentText(curAux);
}

void PlanningDock::onSaveWaypointClicked() {
    bool ok = false;
    QString name = QInputDialog::getText(this, "记录示教点位", "点位名称 (例如 P1, Pick, Place):", QLineEdit::Normal, "P1", &ok);
    if (ok && !name.trimmed().isEmpty()) {
        controller_->saveWaypoint(name.trimmed().toStdString());
    }
}

void PlanningDock::onDeleteWaypointClicked() {
    QListWidgetItem* item = list_waypoints_->currentItem();
    if (item) {
        controller_->deleteWaypoint(item->text().toStdString());
    }
}

void PlanningDock::onMoveToWaypointClicked() {
    QListWidgetItem* item = list_waypoints_->currentItem();
    if (item) {
        controller_->moveToWaypoint(item->text().toStdString());
    }
}

void PlanningDock::onMotionTypeChanged(int index) {
    widget_aux_wp_->setVisible(index == 2); // CIRC
}

void PlanningDock::onStartPlanClicked() {
    const auto& wps = controller_->getWaypoints();

    // 确定起始关节
    std::vector<double> start_joints;
    if (combo_start_wp_->currentIndex() == 0) {
        start_joints = controller_->getCurrentJoints();
    } else {
        std::string sname = combo_start_wp_->currentText().toStdString();
        auto it = wps.find(sname);
        if (it != wps.end()) {
            start_joints = it->second.joint_angles;
        } else {
            QMessageBox::warning(this, "错误", "起始点位未找到！");
            return;
        }
    }

    // 确定目标
    std::string tname = combo_target_wp_->currentText().toStdString();
    auto it_t = wps.find(tname);
    if (it_t == wps.end()) {
        QMessageBox::warning(this, "错误", "目标点位不存在，请先添加点位！");
        return;
    }

    double vscale = spin_vel_scale_->value();
    double ascale = spin_acc_scale_->value();
    double margin = spin_safety_margin_->value();
    double timeout = spin_timeout_->value();

    int mode = combo_motion_type_->currentIndex();
    label_result_banner_->setText("⏳ 正在调用 VAMP SIMD 规划求解引擎，请稍候...");
    label_result_banner_->setStyleSheet("background: #252830; border: 1px solid #faad14; color: #ffd666; padding: 8px 10px; border-radius: 5px;");

    if (mode == 0) { // PTP
        controller_->planPtp(start_joints, it_t->second.joint_angles, vscale, ascale, margin, timeout);
    } else if (mode == 1) { // LIN
        controller_->planLin(start_joints, it_t->second.tcp_pose, vscale, ascale, 0.02, margin);
    } else if (mode == 2) { // CIRC
        std::string aname = combo_aux_wp_->currentText().toStdString();
        auto it_a = wps.find(aname);
        if (it_a == wps.end()) {
            QMessageBox::warning(this, "错误", "圆弧中间点 (Aux) 不存在！");
            return;
        }
        controller_->planCirc(start_joints, it_a->second.tcp_pose, it_t->second.tcp_pose, vscale, ascale, 0.02, margin);
    }
}

void PlanningDock::onTrajectoryGenerated(bool success, int num_points, double duration, const QString& msg) {
    if (success) {
        label_result_banner_->setText(QString("✔ 规划成功: 生成 %1 个关键点, 耗时 %2 秒 (动力学平滑)\n%3")
                                      .arg(num_points).arg(duration, 0, 'f', 2).arg(msg));
        label_result_banner_->setStyleSheet("background: #0d2818; border: 1px solid #169651; color: #00e676; padding: 8px 10px; border-radius: 5px; font-weight: 500;");
    } else {
        label_result_banner_->setText(QString("✖ 规划失败: %1").arg(msg));
        label_result_banner_->setStyleSheet("background: #331111; border: 1px solid #d32f2f; color: #ff5252; padding: 8px 10px; border-radius: 5px; font-weight: 500;");
    }
}

} // namespace sim_app
