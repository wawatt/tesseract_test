#include "add_obstacle_dialog.h"
#include "simulation_controller.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QPushButton>
#include <QFileDialog>
#include <QMessageBox>
#include <cmath>

namespace sim_app {

static void rpyDegToQuat(double roll_deg, double pitch_deg, double yaw_deg, std::vector<double>& quat_out) {
    const double deg2rad = 3.14159265358979323846 / 180.0;
    double cr = std::cos(roll_deg * deg2rad * 0.5);
    double sr = std::sin(roll_deg * deg2rad * 0.5);
    double cp = std::cos(pitch_deg * deg2rad * 0.5);
    double sp = std::sin(pitch_deg * deg2rad * 0.5);
    double cy = std::cos(yaw_deg * deg2rad * 0.5);
    double sy = std::sin(yaw_deg * deg2rad * 0.5);

    quat_out.resize(4);
    quat_out[0] = sr * cp * cy - cr * sp * sy; // qx
    quat_out[1] = cr * sp * cy + sr * cp * sy; // qy
    quat_out[2] = cr * cp * sy - sr * sp * cy; // qz
    quat_out[3] = cr * cp * cy + sr * sp * sy; // qw
}

AddObstacleDialog::AddObstacleDialog(QWidget* parent)
    : QDialog(parent) {
    setWindowTitle("添加场景障碍物 (Add Scene Obstacle)");
    resize(420, 480);
    setupUi();
}

void AddObstacleDialog::setupUi() {
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->setSpacing(10);

    QFormLayout* baseForm = new QFormLayout();
    edit_name_ = new QLineEdit("obstacle_1", this);
    combo_type_ = new QComboBox(this);
    combo_type_->addItem("立方体 (Box)");
    combo_type_->addItem("球体 (Sphere)");
    combo_type_->addItem("圆柱体 (Cylinder)");
    combo_type_->addItem("外部网格文件 (Mesh 3D Model)");

    baseForm->addRow("障碍物名称:", edit_name_);
    baseForm->addRow("几何类型:", combo_type_);
    mainLayout->addLayout(baseForm);

    // 形状参数 Stack
    stack_params_ = new QStackedWidget(this);

    // 0: Box
    QWidget* pageBox = new QWidget();
    QFormLayout* formBox = new QFormLayout(pageBox);
    spin_box_dx_ = new QDoubleSpinBox(pageBox);
    spin_box_dy_ = new QDoubleSpinBox(pageBox);
    spin_box_dz_ = new QDoubleSpinBox(pageBox);
    spin_box_dx_->setRange(0.01, 10.0); spin_box_dx_->setValue(0.5); spin_box_dx_->setSingleStep(0.1);
    spin_box_dy_->setRange(0.01, 10.0); spin_box_dy_->setValue(0.5); spin_box_dy_->setSingleStep(0.1);
    spin_box_dz_->setRange(0.01, 10.0); spin_box_dz_->setValue(0.5); spin_box_dz_->setSingleStep(0.1);
    formBox->addRow("X 长度 (m):", spin_box_dx_);
    formBox->addRow("Y 宽度 (m):", spin_box_dy_);
    formBox->addRow("Z 高度 (m):", spin_box_dz_);
    stack_params_->addWidget(pageBox);

    // 1: Sphere
    QWidget* pageSphere = new QWidget();
    QFormLayout* formSphere = new QFormLayout(pageSphere);
    spin_sphere_r_ = new QDoubleSpinBox(pageSphere);
    spin_sphere_r_->setRange(0.01, 5.0); spin_sphere_r_->setValue(0.2); spin_sphere_r_->setSingleStep(0.05);
    formSphere->addRow("球体半径 (m):", spin_sphere_r_);
    stack_params_->addWidget(pageSphere);

    // 2: Cylinder
    QWidget* pageCyl = new QWidget();
    QFormLayout* formCyl = new QFormLayout(pageCyl);
    spin_cyl_r_ = new QDoubleSpinBox(pageCyl);
    spin_cyl_len_ = new QDoubleSpinBox(pageCyl);
    spin_cyl_r_->setRange(0.01, 5.0); spin_cyl_r_->setValue(0.15); spin_cyl_r_->setSingleStep(0.05);
    spin_cyl_len_->setRange(0.01, 10.0); spin_cyl_len_->setValue(0.6); spin_cyl_len_->setSingleStep(0.1);
    formCyl->addRow("底面半径 (m):", spin_cyl_r_);
    formCyl->addRow("圆柱高度 (m):", spin_cyl_len_);
    stack_params_->addWidget(pageCyl);

    // 3: Mesh
    QWidget* pageMesh = new QWidget();
    QHBoxLayout* meshFileLayout = new QHBoxLayout(pageMesh);
    edit_mesh_path_ = new QLineEdit(pageMesh);
    edit_mesh_path_->setPlaceholderText("选择 .stl / .obj / .dae 模型文件...");
    QPushButton* btnBrowse = new QPushButton("浏览...", pageMesh);
    connect(btnBrowse, &QPushButton::clicked, this, &AddObstacleDialog::onBrowseMeshFile);
    meshFileLayout->addWidget(edit_mesh_path_);
    meshFileLayout->addWidget(btnBrowse);
    stack_params_->addWidget(pageMesh);

    connect(combo_type_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &AddObstacleDialog::onTypeChanged);
    mainLayout->addWidget(stack_params_);

    // 位姿参数
    QGroupBox* poseGroup = new QGroupBox("放置位姿 (World Pose)", this);
    QFormLayout* poseForm = new QFormLayout(poseGroup);
    spin_pos_x_ = new QDoubleSpinBox(poseGroup);
    spin_pos_y_ = new QDoubleSpinBox(poseGroup);
    spin_pos_z_ = new QDoubleSpinBox(poseGroup);
    spin_rot_r_ = new QDoubleSpinBox(poseGroup);
    spin_rot_p_ = new QDoubleSpinBox(poseGroup);
    spin_rot_y_ = new QDoubleSpinBox(poseGroup);

    auto setupSpin = [](QDoubleSpinBox* spin, double minv, double maxv, double val, double step) {
        spin->setRange(minv, maxv);
        spin->setValue(val);
        spin->setSingleStep(step);
    };

    setupSpin(spin_pos_x_, -10.0, 10.0, 1.2, 0.05);
    setupSpin(spin_pos_y_, -10.0, 10.0, 0.0, 0.05);
    setupSpin(spin_pos_z_, -10.0, 10.0, 0.5, 0.05);
    setupSpin(spin_rot_r_, -180.0, 180.0, 0.0, 5.0);
    setupSpin(spin_rot_p_, -180.0, 180.0, 0.0, 5.0);
    setupSpin(spin_rot_y_, -180.0, 180.0, 0.0, 5.0);

    poseForm->addRow("X 位置 (m):", spin_pos_x_);
    poseForm->addRow("Y 位置 (m):", spin_pos_y_);
    poseForm->addRow("Z 位置 (m):", spin_pos_z_);
    poseForm->addRow("Roll 翻滚 (°):", spin_rot_r_);
    poseForm->addRow("Pitch 俯仰 (°):", spin_rot_p_);
    poseForm->addRow("Yaw 偏航 (°):", spin_rot_y_);
    mainLayout->addWidget(poseGroup);

    // 确认 / 取消
    QHBoxLayout* btnLayout = new QHBoxLayout();
    QPushButton* btnOk = new QPushButton("确认添加", this);
    btnOk->setObjectName("btnPrimary");
    QPushButton* btnCancel = new QPushButton("取消", this);

    connect(btnOk, &QPushButton::clicked, this, &QDialog::accept);
    connect(btnCancel, &QPushButton::clicked, this, &QDialog::reject);

    btnLayout->addStretch(1);
    btnLayout->addWidget(btnCancel);
    btnLayout->addWidget(btnOk);
    mainLayout->addLayout(btnLayout);
}

void AddObstacleDialog::onTypeChanged(int index) {
    stack_params_->setCurrentIndex(index);
}

void AddObstacleDialog::onBrowseMeshFile() {
    QString filter = "3D Mesh Files (*.stl *.obj *.dae *.ply);;All Files (*.*)";
    QString path = QFileDialog::getOpenFileName(this, "选择障碍物网格模型", "", filter);
    if (!path.isEmpty()) {
        edit_mesh_path_->setText(path);
    }
}

bool AddObstacleDialog::applyToController(SimulationController* controller) {
    std::string name = edit_name_->text().trimmed().toStdString();
    if (name.empty()) {
        QMessageBox::warning(this, "警告", "障碍物名称不能为空！");
        return false;
    }

    double x = spin_pos_x_->value();
    double y = spin_pos_y_->value();
    double z = spin_pos_z_->value();
    std::vector<double> quat;
    rpyDegToQuat(spin_rot_r_->value(), spin_rot_p_->value(), spin_rot_y_->value(), quat);
    std::vector<double> pose = {x, y, z, quat[0], quat[1], quat[2], quat[3]};

    int typeIdx = combo_type_->currentIndex();
    if (typeIdx == 0) { // Box
        double dx = spin_box_dx_->value();
        double dy = spin_box_dy_->value();
        double dz = spin_box_dz_->value();
        return controller->addBoxObstacle(name, x, y, z, dx, dy, dz);
    } else if (typeIdx == 1) { // Sphere
        double r = spin_sphere_r_->value();
        return controller->addSphereObstacle(name, x, y, z, r);
    } else if (typeIdx == 2) { // Cylinder
        double r = spin_cyl_r_->value();
        double len = spin_cyl_len_->value();
        return controller->addCylinderObstacle(name, r, len, pose);
    } else if (typeIdx == 3) { // Mesh
        std::string filePath = edit_mesh_path_->text().trimmed().toStdString();
        if (filePath.empty()) {
            QMessageBox::warning(this, "警告", "请选择有效的网格模型文件！");
            return false;
        }
        return controller->addMeshObstacle(name, filePath, pose);
    }
    return false;
}

} // namespace sim_app
