#pragma once

#include <QDockWidget>
#include <QListWidget>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QLabel>
#include <QStackedWidget>

namespace sim_app {

class SimulationController;

class PlanningDock : public QDockWidget {
    Q_OBJECT

public:
    explicit PlanningDock(SimulationController* controller, QWidget* parent = nullptr);
    virtual ~PlanningDock() = default;

public slots:
    void onWaypointsChanged();
    void onTrajectoryGenerated(bool success, int num_points, double duration, const QString& msg);

private slots:
    void onSaveWaypointClicked();
    void onDeleteWaypointClicked();
    void onMoveToWaypointClicked();
    void onMotionTypeChanged(int index);
    void onStartPlanClicked();

private:
    void setupUi();
    QWidget* createWaypointsSection();
    QWidget* createPlanningSection();
    void refreshWaypointCombos();

    SimulationController* controller_{nullptr};

    // 点位库控件
    QListWidget* list_waypoints_{nullptr};

    // 规划控件
    QComboBox* combo_motion_type_{nullptr};
    QComboBox* combo_start_wp_{nullptr};
    QComboBox* combo_target_wp_{nullptr};
    QComboBox* combo_aux_wp_{nullptr};
    QWidget* widget_aux_wp_{nullptr};

    QDoubleSpinBox* spin_vel_scale_{nullptr};
    QDoubleSpinBox* spin_acc_scale_{nullptr};
    QDoubleSpinBox* spin_safety_margin_{nullptr};
    QDoubleSpinBox* spin_timeout_{nullptr};

    QPushButton* btn_plan_{nullptr};
    QLabel* label_result_banner_{nullptr};
};

} // namespace sim_app
