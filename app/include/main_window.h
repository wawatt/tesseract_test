#pragma once

#include <QMainWindow>
#include <QLabel>
#include <memory>

namespace sim_app {

class SimulationController;
class OsgViewerWidget;
class SceneTreeDock;
class JogPanelDock;
class PlanningDock;
class TimelineDock;
class LogDock;

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    virtual ~MainWindow();

private slots:
    void onOpenCustomRobot();
    void onReloadDefaultRobot();
    void onAbout();
    void onRobotLoaded(bool success, const QString& robot_name);
    void onJointsUpdated(const std::vector<double>& joints, const std::vector<double>& tcp_pose);

private:
    void setupUi();
    void setupMenus();
    void setupToolBar();
    void setupStatusBar();
    void loadDefaultFanucModelVisuals();

    std::unique_ptr<SimulationController> controller_;

    OsgViewerWidget* viewer_{nullptr};
    SceneTreeDock* dock_scene_tree_{nullptr};
    JogPanelDock* dock_jog_{nullptr};
    PlanningDock* dock_planning_{nullptr};
    TimelineDock* dock_timeline_{nullptr};
    LogDock* dock_log_{nullptr};

    QLabel* label_status_collision_{nullptr};
    QLabel* label_status_tcp_{nullptr};
};

} // namespace sim_app
