#pragma once

#include <QDockWidget>
#include <QTextEdit>
#include <QLabel>
#include <QPushButton>

#include "robot_planner/robot_planner.h"

namespace sim_app {

class SimulationController;

class LogDock : public QDockWidget {
    Q_OBJECT

public:
    explicit LogDock(SimulationController* controller, QWidget* parent = nullptr);
    virtual ~LogDock() = default;

public slots:
    void onLogMessage(int level, const QString& text);
    void onCollisionState(bool in_collision, 
                           const std::vector<std::string>& colliding_links, 
                           const std::vector<robot_planner::ContactInfo>& contacts);

private slots:
    void onClearClicked();

private:
    void setupUi();

    SimulationController* controller_{nullptr};
    QLabel* label_collision_status_{nullptr};
    QTextEdit* text_log_{nullptr};
};

} // namespace sim_app
