#include "timeline_dock.h"
#include "simulation_controller.h"

#include <QVBoxLayout>
#include <QHBoxLayout>

namespace sim_app {

TimelineDock::TimelineDock(SimulationController* controller, QWidget* parent)
    : QDockWidget("轨迹回放时间轴 (Trajectory Playback)", parent), controller_(controller) {
    setObjectName("TimelineDock");
    setupUi();

    connect(controller_, &SimulationController::sigTrajectoryGenerated, this, &TimelineDock::onTrajectoryGenerated);
    connect(controller_, &SimulationController::sigPlaybackTimeChanged, this, &TimelineDock::onPlaybackTimeChanged);
    connect(controller_, &SimulationController::sigPlaybackStateChanged, this, &TimelineDock::onPlaybackStateChanged);
}

void TimelineDock::setupUi() {
    QWidget* container = new QWidget(this);
    QVBoxLayout* mainLayout = new QVBoxLayout(container);
    mainLayout->setContentsMargins(8, 4, 8, 6);
    mainLayout->setSpacing(4);

    // 第一行: 控制按钮与时间轴
    QHBoxLayout* barLayout = new QHBoxLayout();
    barLayout->setSpacing(8);

    btn_play_ = new QPushButton("▶ 播放 (Play)", container);
    btn_play_->setObjectName("btnPrimary");
    btn_play_->setFixedWidth(100);
    btn_play_->setEnabled(false);
    connect(btn_play_, &QPushButton::clicked, this, &TimelineDock::onPlayPauseClicked);
    barLayout->addWidget(btn_play_);

    btn_stop_ = new QPushButton("⏹ 停止", container);
    btn_stop_->setFixedWidth(80);
    btn_stop_->setEnabled(false);
    connect(btn_stop_, &QPushButton::clicked, this, &TimelineDock::onStopClicked);
    barLayout->addWidget(btn_stop_);

    slider_time_ = new QSlider(Qt::Horizontal, container);
    slider_time_->setRange(0, 1000);
    slider_time_->setEnabled(false);
    connect(slider_time_, &QSlider::sliderMoved, this, &TimelineDock::onSliderSeek);
    barLayout->addWidget(slider_time_, 1);

    label_time_ = new QLabel("0.00s / 0.00s", container);
    label_time_->setFixedWidth(115);
    label_time_->setAlignment(Qt::AlignCenter);
    label_time_->setStyleSheet("font-family: 'Consolas', monospace; font-weight: bold; background: #181b22; border: 1px solid #2d3340; border-radius: 4px; padding: 3px 6px; color: #40b0ff;");
    barLayout->addWidget(label_time_);

    combo_speed_ = new QComboBox(container);
    combo_speed_->addItem("0.25x", 0.25);
    combo_speed_->addItem("0.5x", 0.5);
    combo_speed_->addItem("1.0x (原速)", 1.0);
    combo_speed_->addItem("2.0x", 2.0);
    combo_speed_->addItem("5.0x", 5.0);
    combo_speed_->setCurrentIndex(2);
    connect(combo_speed_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &TimelineDock::onSpeedChanged);
    barLayout->addWidget(combo_speed_);

    chk_loop_ = new QCheckBox("循环 (Loop)", container);
    connect(chk_loop_, &QCheckBox::toggled, this, &TimelineDock::onLoopToggled);
    barLayout->addWidget(chk_loop_);

    mainLayout->addLayout(barLayout);

    // 第二行: 信息提示
    label_info_ = new QLabel("无可用轨迹 - 请在规划面板生成一条轨迹", container);
    label_info_->setStyleSheet("color: #7b8294; font-size: 11px;");
    mainLayout->addWidget(label_info_);

    setWidget(container);
}

void TimelineDock::onTrajectoryGenerated(bool success, int num_points, double duration, const QString& msg) {
    (void)msg;
    if (success && num_points > 0) {
        total_duration_ = duration;
        btn_play_->setEnabled(true);
        btn_stop_->setEnabled(true);
        slider_time_->setEnabled(true);
        label_time_->setText(QString("0.00s / %1s").arg(duration, 0, 'f', 2));
        label_info_->setText(QString("✔ 当前已就绪轨迹: %1 个关键点, 持续时间 %2 秒 (动力学平滑)")
                             .arg(num_points).arg(duration, 0, 'f', 2));
        label_info_->setStyleSheet("color: #00e676; font-size: 11px;");
    } else {
        total_duration_ = 0.0;
        btn_play_->setEnabled(false);
        btn_stop_->setEnabled(false);
        slider_time_->setEnabled(false);
        label_info_->setText("✖ 轨迹规划失败");
        label_info_->setStyleSheet("color: #f44336; font-size: 11px;");
    }
}

void TimelineDock::onPlaybackTimeChanged(double current_time, double total_duration) {
    if (user_seeking_) return;
    total_duration_ = total_duration;

    if (total_duration > 0.0) {
        int pos = static_cast<int>((current_time / total_duration) * 1000.0);
        slider_time_->setValue(pos);
        label_time_->setText(QString("%1s / %2s")
                             .arg(current_time, 5, 'f', 2)
                             .arg(total_duration, 5, 'f', 2));
    }
}

void TimelineDock::onPlaybackStateChanged(bool is_playing) {
    if (is_playing) {
        btn_play_->setText("⏸ 暂停 (Pause)");
    } else {
        btn_play_->setText("▶ 播放 (Play)");
    }
}

void TimelineDock::onPlayPauseClicked() {
    if (controller_->isPlaying()) {
        controller_->pause();
    } else {
        controller_->play();
    }
}

void TimelineDock::onStopClicked() {
    controller_->stop();
}

void TimelineDock::onSliderSeek(int int_val) {
    if (total_duration_ <= 0.0) return;
    user_seeking_ = true;
    double t = (static_cast<double>(int_val) / 1000.0) * total_duration_;
    controller_->seek(t);
    user_seeking_ = false;
}

void TimelineDock::onSpeedChanged(int index) {
    double spd = combo_speed_->itemData(index).toDouble();
    controller_->setPlaybackSpeed(spd);
}

void TimelineDock::onLoopToggled(bool checked) {
    controller_->setLooping(checked);
}

} // namespace sim_app
