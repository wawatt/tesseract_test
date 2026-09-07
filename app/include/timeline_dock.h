#pragma once

#include <QDockWidget>
#include <QPushButton>
#include <QSlider>
#include <QLabel>
#include <QComboBox>
#include <QCheckBox>

namespace sim_app {

class SimulationController;

class TimelineDock : public QDockWidget {
    Q_OBJECT

public:
    explicit TimelineDock(SimulationController* controller, QWidget* parent = nullptr);
    virtual ~TimelineDock() = default;

public slots:
    void onTrajectoryGenerated(bool success, int num_points, double duration, const QString& msg);
    void onPlaybackTimeChanged(double current_time, double total_duration);
    void onPlaybackStateChanged(bool is_playing);

private slots:
    void onPlayPauseClicked();
    void onStopClicked();
    void onSliderSeek(int int_val);
    void onSpeedChanged(int index);
    void onLoopToggled(bool checked);

private:
    void setupUi();

    SimulationController* controller_{nullptr};

    QPushButton* btn_play_{nullptr};
    QPushButton* btn_stop_{nullptr};
    QSlider* slider_time_{nullptr};
    QLabel* label_time_{nullptr};
    QLabel* label_info_{nullptr};
    QComboBox* combo_speed_{nullptr};
    QCheckBox* chk_loop_{nullptr};

    bool user_seeking_{false};
    double total_duration_{0.0};
};

} // namespace sim_app
