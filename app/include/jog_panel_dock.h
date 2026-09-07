#pragma once

#include <QDockWidget>
#include <QSlider>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QComboBox>
#include <QCheckBox>
#include <vector>

namespace sim_app {

class SimulationController;

class JogPanelDock : public QDockWidget {
    Q_OBJECT

public:
    explicit JogPanelDock(SimulationController* controller, QWidget* parent = nullptr);
    virtual ~JogPanelDock() = default;

public slots:
    void onJointsUpdated(const std::vector<double>& joints, const std::vector<double>& tcp_pose);
    void onRobotLoaded(bool success, const QString& robot_name);
    void onCartesianJogFeedback(int status_code, const QString& message);

private slots:
    void onSliderValueChanged(int joint_idx, int int_val);
    void onSpinBoxValueChanged(int joint_idx, double val_deg);
    void onCartesianStepClicked(int axis, bool positive);

private:
    void setupUi();
    QWidget* createJointJogTab();
    QWidget* createCartesianJogTab();

    SimulationController* controller_{nullptr};

    // 关节控件
    struct JointControlRow {
        QLabel* label;
        QSlider* slider;
        QDoubleSpinBox* spinbox;
        double lower_deg{-180.0};
        double upper_deg{180.0};
    };
    std::vector<JointControlRow> joint_rows_;

    // 笛卡尔控件与数字仪表
    QLabel* label_val_x_{nullptr};
    QLabel* label_val_y_{nullptr};
    QLabel* label_val_z_{nullptr};
    QLabel* label_val_rx_{nullptr};
    QLabel* label_val_ry_{nullptr};
    QLabel* label_val_rz_{nullptr};
    QComboBox* combo_trans_step_{nullptr};
    QComboBox* combo_rot_step_{nullptr};
    QLabel* label_jog_feedback_{nullptr};
    QCheckBox* chk_stop_on_collision_{nullptr};

    bool updating_ui_{false};
};

} // namespace sim_app
