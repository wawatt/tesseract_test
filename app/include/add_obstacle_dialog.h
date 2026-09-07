#pragma once

#include <QDialog>
#include <QLineEdit>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QStackedWidget>

namespace sim_app {

class SimulationController;

class AddObstacleDialog : public QDialog {
    Q_OBJECT

public:
    explicit AddObstacleDialog(QWidget* parent = nullptr);
    virtual ~AddObstacleDialog() = default;

    bool applyToController(SimulationController* controller);

private slots:
    void onTypeChanged(int index);
    void onBrowseMeshFile();

private:
    void setupUi();

    QLineEdit* edit_name_{nullptr};
    QComboBox* combo_type_{nullptr};
    QStackedWidget* stack_params_{nullptr};

    // Box params
    QDoubleSpinBox* spin_box_dx_{nullptr};
    QDoubleSpinBox* spin_box_dy_{nullptr};
    QDoubleSpinBox* spin_box_dz_{nullptr};

    // Sphere params
    QDoubleSpinBox* spin_sphere_r_{nullptr};

    // Cylinder params
    QDoubleSpinBox* spin_cyl_r_{nullptr};
    QDoubleSpinBox* spin_cyl_len_{nullptr};

    // Mesh params
    QLineEdit* edit_mesh_path_{nullptr};

    // Pose params
    QDoubleSpinBox* spin_pos_x_{nullptr};
    QDoubleSpinBox* spin_pos_y_{nullptr};
    QDoubleSpinBox* spin_pos_z_{nullptr};
    QDoubleSpinBox* spin_rot_r_{nullptr};
    QDoubleSpinBox* spin_rot_p_{nullptr};
    QDoubleSpinBox* spin_rot_y_{nullptr};
};

} // namespace sim_app
