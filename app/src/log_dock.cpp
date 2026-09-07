#include "log_dock.h"
#include "simulation_controller.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QDateTime>

namespace sim_app {

LogDock::LogDock(SimulationController* controller, QWidget* parent)
    : QDockWidget("系统日志与诊断 (System Diagnostics)", parent), controller_(controller) {
    setObjectName("LogDock");
    setupUi();

    connect(controller_, &SimulationController::sigLogMessage, this, &LogDock::onLogMessage);
    connect(controller_, &SimulationController::sigCollisionState, this, &LogDock::onCollisionState);
}

void LogDock::setupUi() {
    QWidget* container = new QWidget(this);
    QVBoxLayout* mainLayout = new QVBoxLayout(container);
    mainLayout->setContentsMargins(6, 6, 6, 6);
    mainLayout->setSpacing(6);

    // 碰撞预警状态条
    label_collision_status_ = new QLabel("安全 - 实时干涉质检在线 (无碰撞)", container);
    label_collision_status_->setStyleSheet("background: #1b5e20; color: #ffffff; padding: 4px 8px; border-radius: 4px; font-weight: bold;");
    mainLayout->addWidget(label_collision_status_);

    text_log_ = new QTextEdit(container);
    text_log_->setReadOnly(true);
    text_log_->setStyleSheet("background-color: #141518; color: #d0d4e0; font-family: 'Consolas', monospace; font-size: 12px;");
    mainLayout->addWidget(text_log_, 1);

    QHBoxLayout* btnLayout = new QHBoxLayout();
    QPushButton* btnClear = new QPushButton("清空控制台", container);
    btnClear->setFixedWidth(90);
    connect(btnClear, &QPushButton::clicked, this, &LogDock::onClearClicked);
    btnLayout->addStretch(1);
    btnLayout->addWidget(btnClear);
    mainLayout->addLayout(btnLayout);

    setWidget(container);
}

void LogDock::onLogMessage(int level, const QString& text) {
    QString timeStr = QDateTime::currentDateTime().toString("hh:mm:ss.zzz");
    QString color = "#cfd2dc";
    QString tag = "INFO";

    if (level == 1) {
        color = "#00e676";
        tag = "SUCCESS";
    } else if (level == 2) {
        color = "#ffb300";
        tag = "WARN";
    } else if (level == 3) {
        color = "#ff5252";
        tag = "ERROR";
    }

    QString html = QString("<span style='color:#60687a;'>[%1]</span> <span style='color:%2; font-weight:bold;'>[%3]</span> <span style='color:%2;'>%4</span>")
                   .arg(timeStr, color, tag, text.toHtmlEscaped());

    text_log_->append(html);
}

void LogDock::onCollisionState(bool in_collision, 
                               const std::vector<std::string>& colliding_links, 
                               const std::vector<robot_planner::ContactInfo>& contacts) {
    if (in_collision) {
        QString linkStr;
        for (size_t i = 0; i < colliding_links.size(); ++i) {
            if (i > 0) linkStr += ", ";
            linkStr += QString::fromStdString(colliding_links[i]);
        }

        double min_dist = 0.0;
        if (!contacts.empty()) {
            min_dist = contacts.front().distance;
            for (const auto& c : contacts) {
                if (c.distance < min_dist) min_dist = c.distance;
            }
        }

        label_collision_status_->setText(QString("警告: 检测到碰撞干涉! 涉及连杆: [%1] (最小侵入深度: %2 mm)")
                                         .arg(linkStr)
                                         .arg(min_dist * 1000.0, 0, 'f', 1));
        label_collision_status_->setStyleSheet("background: #b71c1c; color: #ffffff; padding: 4px 8px; border-radius: 4px; font-weight: bold;");
    } else {
        label_collision_status_->setText("安全 - 实时干涉质检在线 (无碰撞)");
        label_collision_status_->setStyleSheet("background: #1b5e20; color: #ffffff; padding: 4px 8px; border-radius: 4px; font-weight: bold;");
    }
}

void LogDock::onClearClicked() {
    text_log_->clear();
}

} // namespace sim_app
