#include "main_window.h"
#include "simulation_controller.h"
#include "osg_viewer_widget.h"
#include "scene_tree_dock.h"
#include "jog_panel_dock.h"
#include "planning_dock.h"
#include "timeline_dock.h"
#include "log_dock.h"

#include <QMenuBar>
#include <QToolBar>
#include <QStatusBar>
#include <QFileDialog>
#include <QMessageBox>
#include <QTimer>
#include <iostream>
#include <filesystem>

namespace sim_app {

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent), controller_(std::make_unique<SimulationController>(this)) {
    setWindowTitle("工业机器人三维运动仿真与交互式示教工作站 - Robot Simulation Workstation");
    resize(1600, 960);

    setupUi();
    setupMenus();
    setupToolBar();
    setupStatusBar();

    connect(controller_.get(), &SimulationController::sigRobotLoaded, this, &MainWindow::onRobotLoaded);
    connect(controller_.get(), &SimulationController::sigJointsUpdated, this, &MainWindow::onJointsUpdated);
    connect(controller_.get(), &SimulationController::sigCollisionState, this, 
            [this](bool in_coll, const std::vector<std::string>& clinks, const std::vector<robot_planner::ContactInfo>& contacts) {
                viewer_->getRobotVisualNode()->setCollisionHighlight(clinks);
                viewer_->updateContactMarkers(contacts);
                if (label_status_collision_) {
                    if (in_coll) {
                        label_status_collision_->setText(QString("🔴 碰撞报警: 发生干涉 (%1 连杆)").arg(clinks.size()));
                        label_status_collision_->setStyleSheet("background-color: #4a1515; border: 1px solid #ff4d4f; color: #ff7875; border-radius: 4px; padding: 2px 8px; font-weight: bold;");
                    } else {
                        label_status_collision_->setText("🛡️ 碰撞防护: 安全 (实时质检在线)");
                        label_status_collision_->setStyleSheet("background-color: #14281a; border: 1px solid #237804; color: #73d13d; border-radius: 4px; padding: 2px 8px; font-weight: bold;");
                    }
                }
            });

    // 初始化默认机械臂与 3D 视觉模型
    onReloadDefaultRobot();
}

MainWindow::~MainWindow() = default;

void MainWindow::setupUi() {
    setDockOptions(QMainWindow::AnimatedDocks | QMainWindow::AllowTabbedDocks | QMainWindow::AllowNestedDocks);

    // 中心视口
    viewer_ = new OsgViewerWidget(this);
    setCentralWidget(viewer_);

    // 1. 左侧: 场景与障碍物树
    dock_scene_tree_ = new SceneTreeDock(controller_.get(), viewer_, this);
    addDockWidget(Qt::LeftDockWidgetArea, dock_scene_tree_);

    // 2. 右侧: 示教与规划面板 (Tabified 智能标签化合并，各独享 100% 高度空间)
    dock_jog_ = new JogPanelDock(controller_.get(), this);
    addDockWidget(Qt::RightDockWidgetArea, dock_jog_);

    dock_planning_ = new PlanningDock(controller_.get(), this);
    addDockWidget(Qt::RightDockWidgetArea, dock_planning_);
    tabifyDockWidget(dock_jog_, dock_planning_);
    dock_jog_->raise(); // 默认先展示示教面板

    // 3. 底部: 时间轴回放
    dock_timeline_ = new TimelineDock(controller_.get(), this);
    addDockWidget(Qt::BottomDockWidgetArea, dock_timeline_);

    // 4. 底部: 诊断日志 (与时间轴合并 Tab)
    dock_log_ = new LogDock(controller_.get(), this);
    addDockWidget(Qt::BottomDockWidgetArea, dock_log_);
    tabifyDockWidget(dock_timeline_, dock_log_);
    dock_timeline_->raise();
}

void MainWindow::setupMenus() {
    QMenuBar* mb = menuBar();

    // 文件菜单
    QMenu* menuFile = mb->addMenu("文件 (&File)");
    QAction* actOpen = menuFile->addAction("打开自定义 URDF/SRDF 模型...");
    actOpen->setShortcut(QKeySequence::Open);
    connect(actOpen, &QAction::triggered, this, &MainWindow::onOpenCustomRobot);

    QAction* actReload = menuFile->addAction("重新加载预置 Fanuc R-2000iC 模型");
    connect(actReload, &QAction::triggered, this, &MainWindow::onReloadDefaultRobot);

    menuFile->addSeparator();
    QAction* actExit = menuFile->addAction("退出 (&Exit)");
    actExit->setShortcut(QKeySequence::Quit);
    connect(actExit, &QAction::triggered, this, &QWidget::close);

    // 视图菜单
    QMenu* menuView = mb->addMenu("视图 (&View)");
    QAction* actResetCam = menuView->addAction("重置视口观察相机");
    connect(actResetCam, &QAction::triggered, viewer_, &OsgViewerWidget::resetCameraView);

    QAction* actGrid = menuView->addAction("显示地表网格");
    actGrid->setCheckable(true);
    actGrid->setChecked(true);
    connect(actGrid, &QAction::toggled, viewer_, &OsgViewerWidget::setGridVisible);

    QAction* actAxes = menuView->addAction("显示世界坐标系三轴");
    actAxes->setCheckable(true);
    actAxes->setChecked(true);
    connect(actAxes, &QAction::toggled, viewer_, &OsgViewerWidget::setAxesVisible);

    QAction* actToolFrame = menuView->addAction("显示 TCP 末端坐标系");
    actToolFrame->setCheckable(true);
    actToolFrame->setChecked(true);
    connect(actToolFrame, &QAction::toggled, this, [this](bool visible) {
        viewer_->getRobotVisualNode()->showToolCoordinateFrame(visible);
        viewer_->update();
    });

    menuView->addSeparator();
    menuView->addAction(dock_scene_tree_->toggleViewAction());
    menuView->addAction(dock_jog_->toggleViewAction());
    menuView->addAction(dock_planning_->toggleViewAction());
    menuView->addAction(dock_timeline_->toggleViewAction());
    menuView->addAction(dock_log_->toggleViewAction());

    // 帮助菜单
    QMenu* menuHelp = mb->addMenu("帮助 (&Help)");
    QAction* actAbout = menuHelp->addAction("关于仿真软件...");
    connect(actAbout, &QAction::triggered, this, &MainWindow::onAbout);
}

void MainWindow::setupToolBar() {
    QToolBar* tb = addToolBar("快捷控制工具栏 (Tools)");
    tb->setMovable(false);

    QAction* actHome = tb->addAction("🏠 归位 (Home)");
    connect(actHome, &QAction::triggered, this, [this]() {
        controller_->moveToWaypoint("Home");
    });

    QAction* actReady = tb->addAction("⚡ 就绪位 (Ready)");
    connect(actReady, &QAction::triggered, this, [this]() {
        controller_->moveToWaypoint("Ready");
    });

    tb->addSeparator();

    QAction* actPlay = tb->addAction("▶/⏸ 播放/暂停轨迹");
    connect(actPlay, &QAction::triggered, this, [this]() {
        if (controller_->isPlaying()) controller_->pause();
        else controller_->play();
    });

    tb->addSeparator();

    QAction* actResetCam = tb->addAction("🎥 重置相机视角");
    connect(actResetCam, &QAction::triggered, viewer_, &OsgViewerWidget::resetCameraView);

    tb->addSeparator();

    QAction* actAddObs = tb->addAction("➕ 添加障碍物");
    connect(actAddObs, &QAction::triggered, dock_scene_tree_, &SceneTreeDock::onAddObstacleClicked);
}

void MainWindow::setupStatusBar() {
    QStatusBar* sb = statusBar();

    label_status_tcp_ = new QLabel("📍 TCP: [0.000, 0.000, 0.000] m", this);
    label_status_tcp_->setObjectName("statusChip");
    label_status_tcp_->setStyleSheet("font-family: 'Consolas', monospace; color: #40b0ff; background-color: #1a1d24; border: 1px solid #2d3340; border-radius: 4px; padding: 2px 8px; font-weight: bold; margin-left: 4px;");
    sb->addWidget(label_status_tcp_);

    label_status_collision_ = new QLabel("🛡️ 碰撞防护: 安全 (实时质检在线)", this);
    label_status_collision_->setObjectName("statusChip");
    label_status_collision_->setStyleSheet("background-color: #14281a; border: 1px solid #237804; color: #73d13d; border-radius: 4px; padding: 2px 8px; font-weight: bold; margin-right: 8px;");
    sb->addPermanentWidget(label_status_collision_);

    QLabel* labelBackend = new QLabel("🟢 规划后端: VAMP SIMD (AVX2)", this);
    labelBackend->setObjectName("statusChip");
    labelBackend->setStyleSheet("background-color: #141f2e; border: 1px solid #1765ad; color: #40b0ff; border-radius: 4px; padding: 2px 8px; font-weight: bold; margin-right: 8px;");
    sb->addPermanentWidget(labelBackend);

    QLabel* labelFps = new QLabel("🔵 渲染引擎: OSG 3.6.5", this);
    labelFps->setObjectName("statusChip");
    labelFps->setStyleSheet("background-color: #1a1d24; border: 1px solid #2d3340; color: #8c93a4; border-radius: 4px; padding: 2px 8px; margin-right: 4px;");
    sb->addPermanentWidget(labelFps);
}

static std::string resolveMeshPath(const std::string& relPath) {
    if (std::filesystem::exists(relPath)) return std::filesystem::absolute(relPath).string();
    if (std::filesystem::exists("../" + relPath)) return std::filesystem::absolute("../" + relPath).string();
    if (std::filesystem::exists("../../" + relPath)) return std::filesystem::absolute("../../" + relPath).string();
    return relPath;
}

void MainWindow::loadDefaultFanucModelVisuals() {
    std::vector<LinkMeshInfo> links;
    links.push_back({"base_link", resolveMeshPath("vamp_r2000ic/models/meshes/r2000ic_165f/visual/base.dae")});
    links.push_back({"J1_link",   resolveMeshPath("vamp_r2000ic/models/meshes/r2000ic_165f/visual/j1.dae")});
    links.push_back({"J2_link",   resolveMeshPath("vamp_r2000ic/models/meshes/r2000ic_165f/visual/j2.dae")});
    links.push_back({"J3_link",   resolveMeshPath("vamp_r2000ic/models/meshes/r2000ic_165f/visual/j3.dae")});
    links.push_back({"J4_link",   resolveMeshPath("vamp_r2000ic/models/meshes/r2000ic_165f/visual/j4.dae")});
    links.push_back({"J5_link",   resolveMeshPath("vamp_r2000ic/models/meshes/r2000ic_165f/visual/j5.dae")});
    links.push_back({"J6_link",   resolveMeshPath("vamp_r2000ic/models/meshes/r2000ic_165f/visual/j6.dae")});
    links.push_back({"tool0",     ""}); // tool0 无实体网格，附带坐标三轴

    for (const auto& l : links) {
        std::cout << "[MainWindow] Link: " << l.link_name << " -> " << l.mesh_path 
                  << " (exists: " << (!l.mesh_path.empty() && std::filesystem::exists(l.mesh_path)) << ")" << std::endl;
    }

    if (viewer_ && viewer_->getRobotVisualNode()) {
        viewer_->getRobotVisualNode()->buildRobotModel(links);
        viewer_->resetCameraView();
    }
}

void MainWindow::onReloadDefaultRobot() {
    std::cout << "[MainWindow] onReloadDefaultRobot() triggered" << std::endl;
    loadDefaultFanucModelVisuals();
    controller_->initDefaultRobot();
    onJointsUpdated(controller_->getCurrentJoints(), controller_->getCurrentTcpPose());
}

void MainWindow::onOpenCustomRobot() {
    QString urdfPath = QFileDialog::getOpenFileName(this, "选择 URDF 机器人描述文件", "", "URDF Files (*.urdf);;All Files (*.*)");
    if (urdfPath.isEmpty()) return;

    QString srdfPath = QFileDialog::getOpenFileName(this, "选择 SRDF 语义配置文件", "", "SRDF Files (*.srdf);;All Files (*.*)");
    if (srdfPath.isEmpty()) return;

    // 默认加载
    controller_->loadRobot(urdfPath.toStdString(), srdfPath.toStdString(), "manipulator", "base_link", "tool0");
}

void MainWindow::onRobotLoaded(bool success, const QString& robot_name) {
    if (success) {
        statusBar()->showMessage(QString("当前生效机器人: %1").arg(robot_name), 5000);
        onJointsUpdated(controller_->getCurrentJoints(), controller_->getCurrentTcpPose());
    } else {
        statusBar()->showMessage("机器人加载失败，详情请查看诊断日志", 5000);
    }
}

void MainWindow::onJointsUpdated(const std::vector<double>& joints, const std::vector<double>& tcp_pose) {
    (void)joints;
    if (label_status_tcp_ && tcp_pose.size() >= 3) {
        label_status_tcp_->setText(QString("📍 TCP: [%1, %2, %3] m")
                                   .arg(tcp_pose[0], 6, 'f', 3)
                                   .arg(tcp_pose[1], 6, 'f', 3)
                                   .arg(tcp_pose[2], 6, 'f', 3));
    }
    std::vector<std::string> names;
    std::vector<std::vector<double>> poses;
    if (controller_->getLinkWorldPoses(names, poses)) {
        static bool printed_once = false;
        if (!printed_once) {
            printed_once = true;
            std::cout << "===== LINK WORLD POSES =====" << std::endl;
            for (size_t i = 0; i < names.size(); ++i) {
                std::cout << "  " << names[i] << " -> Pos: [" 
                          << poses[i][0] << ", " << poses[i][1] << ", " << poses[i][2]
                          << "] Quat: [" << poses[i][3] << ", " << poses[i][4] 
                          << ", " << poses[i][5] << ", " << poses[i][6] << "]" << std::endl;
            }
            std::cout << "============================" << std::endl;
        }
        viewer_->getRobotVisualNode()->updateLinkPoses(names, poses);
        viewer_->update();
    }
}

void MainWindow::onAbout() {
    QMessageBox::about(this, "关于仿真软件",
        "<h3>工业机器人 3D 仿真与示教工作站</h3>"
        "<p>基于 <b>Qt5</b>、<b>OpenSceneGraph (OSG)</b> 与 <b>robot_planner</b> 深度集成构建。</p>"
        "<p>特性：</p>"
        "<ul>"
        "<li>VAMP AVX2 SIMD 超高速运动学与避障规划加速</li>"
        "<li>PTP 自由避障 / LIN 直线 / CIRC 圆弧全运动指令支持</li>"
        "<li>单轴关节与笛卡尔末端 Jogging 交互式示教</li>"
        "<li>场景障碍物管理、工件抓取挂载 (Attach/Detach)</li>"
        "<li>实时干涉冲突 3D 变红高亮与接触点法线诊断</li>"
        "</ul>");
}

} // namespace sim_app
