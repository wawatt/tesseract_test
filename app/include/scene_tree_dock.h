#pragma once

#include <QDockWidget>
#include <QTreeWidget>
#include <QPushButton>

namespace sim_app {

class SimulationController;
class OsgViewerWidget;

class SceneTreeDock : public QDockWidget {
    Q_OBJECT

public:
    explicit SceneTreeDock(SimulationController* controller, OsgViewerWidget* viewer, QWidget* parent = nullptr);
    virtual ~SceneTreeDock() = default;

public slots:
    void onRobotLoaded(bool success, const QString& robot_name);
    void onObstaclesChanged();
    void onCollisionState(bool in_collision, const std::vector<std::string>& colliding_links);
    void onAddObstacleClicked();
    void onClearObstaclesClicked();

private slots:
    void onCustomContextMenuRequested(const QPoint& pos);

private:
    void setupUi();
    void updateObstacleVisuals();

    SimulationController* controller_{nullptr};
    OsgViewerWidget* viewer_{nullptr};

    QTreeWidget* tree_widget_{nullptr};
    QTreeWidgetItem* item_robot_root_{nullptr};
    QTreeWidgetItem* item_obstacles_root_{nullptr};
};

} // namespace sim_app
