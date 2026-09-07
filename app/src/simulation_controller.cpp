#include "simulation_controller.h"
#include "mesh_loader.h"

#include <iostream>
#include <cmath>
#include <algorithm>
#include <cstdlib>
#include <filesystem>

namespace sim_app {

SimulationController::SimulationController(QObject* parent)
    : QObject(parent), planner_(std::make_unique<robot_planner::RobotPlanner>()) {
    connect(&playback_timer_, &QTimer::timeout, this, &SimulationController::onPlaybackTick);
}

SimulationController::~SimulationController() {
    playback_timer_.stop();
}

static std::string resolvePath(const std::string& relPath) {
    if (std::filesystem::exists(relPath)) return std::filesystem::absolute(relPath).string();
    if (std::filesystem::exists("../" + relPath)) return std::filesystem::absolute("../" + relPath).string();
    if (std::filesystem::exists("../../" + relPath)) return std::filesystem::absolute("../../" + relPath).string();
    return relPath;
}

bool SimulationController::initDefaultRobot() {
    _putenv_s("TESSERACT_RESOURCE_PATH", "D:/build/vcpkg/vcpkg_installed/x64-windows/share");

    const std::string urdf = resolvePath("vamp_r2000ic/models/r2000ic_165f.urdf");
    const std::string srdf = resolvePath("vamp_r2000ic/models/r2000ic_165f.srdf");
    const std::string manip = "r2000ic_165f";
    const std::string base = "base_link";
    const std::string tool = "tool0";

    return loadRobot(urdf, srdf, manip, base, tool);
}

bool SimulationController::loadRobot(const std::string& urdf_path, 
                                     const std::string& srdf_path,
                                     const std::string& manip_name,
                                     const std::string& base_link,
                                     const std::string& tool_link) {
    emit sigLogMessage(0, QString("正在初始化规划器: %1 ...").arg(QString::fromStdString(manip_name)));

    robot_loaded_ = false;
    base_link_ = base_link;
    tool_link_ = tool_link;
    robot_name_ = manip_name;

    if (!planner_->init(urdf_path, srdf_path, manip_name, base_link, tool_link, robot_planner::PlannerBackend::VAMP)) {
        QString err = QString::fromStdString(planner_->getLastError());
        emit sigLogMessage(3, QString("机械臂初始化失败: %1").arg(err));
        emit sigRobotLoaded(false, QString::fromStdString(manip_name));
        return false;
    }

    // 默认配置 6 轴 Fanuc R-2000iC 限位 (如果动态解析，可由 URDF 解析覆盖)
    joint_names_ = {"J1", "J2", "J3", "J4", "J5", "J6"};
    link_names_ = {"base_link", "J1_link", "J2_link", "J3_link", "J4_link", "J5_link", "J6_link", tool_link_};
    joint_lower_ = {-3.228859, -1.047197, -1.378810, -6.283185, -2.181661, -6.283185};
    joint_upper_ = { 3.228859,  1.326450,  3.141592,  6.283185,  2.181661,  6.283185};
    current_joints_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    current_tcp_pose_.resize(7, 0.0);
    planner_->computeFK(current_joints_, current_tcp_pose_);

    robot_loaded_ = true;

    // 预置默认示教点位
    waypoints_.clear();
    Waypoint wpHome;
    wpHome.name = "Home";
    wpHome.joint_angles = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    wpHome.tcp_pose = current_tcp_pose_;
    waypoints_["Home"] = wpHome;

    Waypoint wpReady;
    wpReady.name = "Ready";
    wpReady.joint_angles = {0.0, 0.35, -0.45, 0.0, -0.65, 0.0};
    planner_->computeFK(wpReady.joint_angles, wpReady.tcp_pose);
    waypoints_["Ready"] = wpReady;

    emit sigLogMessage(1, QString("机械臂加载就绪: %1 (VAMP SIMD 加速模式)").arg(QString::fromStdString(manip_name)));
    emit sigRobotLoaded(true, QString::fromStdString(manip_name));
    emit sigJointsUpdated(current_joints_, current_tcp_pose_);
    emit sigWaypointsChanged();

    performCollisionCheck();
    return true;
}

bool SimulationController::getLinkWorldPoses(std::vector<std::string>& names_out, 
                                             std::vector<std::vector<double>>& poses_out) {
    if (!robot_loaded_) return false;
    names_out = link_names_;
    poses_out.resize(link_names_.size());

    for (size_t i = 0; i < link_names_.size(); ++i) {
        poses_out[i].resize(7, 0.0);
        planner_->computeFKForLink(current_joints_, link_names_[i], poses_out[i]);
    }
    return true;
}

bool SimulationController::setJoints(const std::vector<double>& joints) {
    if (!robot_loaded_ || joints.size() != current_joints_.size()) return false;

    current_joints_ = joints;
    for (size_t i = 0; i < current_joints_.size(); ++i) {
        current_joints_[i] = std::clamp(current_joints_[i], joint_lower_[i], joint_upper_[i]);
    }

    planner_->computeFK(current_joints_, current_tcp_pose_);
    emit sigJointsUpdated(current_joints_, current_tcp_pose_);
    performCollisionCheck();
    return true;
}

bool SimulationController::jogJoint(size_t joint_idx, double delta_rad) {
    if (!robot_loaded_ || joint_idx >= current_joints_.size()) return false;
    auto joints = current_joints_;
    joints[joint_idx] += delta_rad;
    return setJoints(joints);
}

bool SimulationController::jogCartesian(int axis_idx, double delta_val, bool stop_on_collision) {
    if (!robot_loaded_) return false;

    auto target_pose = current_tcp_pose_;
    if (axis_idx >= 0 && axis_idx <= 2) {
        target_pose[axis_idx] += delta_val;
    } else if (axis_idx >= 3 && axis_idx <= 5) {
        // 简易小角旋转变换更新四元数
        double angle = delta_val;
        double half = angle * 0.5;
        double s = std::sin(half);
        double c = std::cos(half);
        double dqx = (axis_idx == 3 ? s : 0.0);
        double dqy = (axis_idx == 4 ? s : 0.0);
        double dqz = (axis_idx == 5 ? s : 0.0);
        double dqw = c;

        // 四元数相乘 q_new = dq * q_old
        double ox = target_pose[3], oy = target_pose[4], oz = target_pose[5], ow = target_pose[6];
        target_pose[3] = dqw * ox + dqx * ow + dqy * oz - dqz * oy;
        target_pose[4] = dqw * oy - dqx * oz + dqy * ow + dqz * ox;
        target_pose[5] = dqw * oz + dqx * oy - dqy * ox + dqz * ow;
        target_pose[6] = dqw * ow - dqx * ox - dqy * oy - dqz * oz;
    }

    std::vector<double> ik_joints;
    // 1. 逆运动学求解
    if (!planner_->computeIK(target_pose, current_joints_, ik_joints)) {
        // === 明确判断：逆解求解失败（工作空间限位或奇异点），未发生碰撞 ===
        QString msg = QString("⚠️ 逆解失败: 目标位姿无有效运动学解 (超出工作空间极限或处于奇异点)");
        emit sigLogMessage(2, msg);
        emit sigCartesianJogFeedback(1, msg);
        return false;
    }

    // 2. 逆解成功！检测目标位姿是否存在物理碰撞干涉
    std::vector<robot_planner::ContactInfo> contacts;
    bool in_collision = planner_->checkCollisionDetailed(ik_joints, contacts, 0.0);

    if (in_collision) {
        // === 明确判断：逆解成功，但检测到碰撞 ===
        QString coll_details;
        for (const auto& c : contacts) {
            if (c.distance <= 0.001) {
                QString l1 = QString::fromStdString(c.link_name1.empty() ? "World" : c.link_name1);
                QString l2 = QString::fromStdString(c.link_name2.empty() ? "Obstacle" : c.link_name2);
                coll_details = QString("%1 <-> %2").arg(l1, l2);
                break;
            }
        }
        if (coll_details.isEmpty() && !contacts.empty()) {
            coll_details = QString::fromStdString(contacts[0].link_name1 + " <-> " + contacts[0].link_name2);
        }

        if (stop_on_collision) {
            QString msg = QString("🚫 碰撞拦截: 逆解成功，但目标位置与 [%1] 发生干涉！已阻止运动").arg(coll_details.isEmpty() ? "障碍物" : coll_details);
            emit sigLogMessage(3, msg);
            emit sigCartesianJogFeedback(2, msg);
            return false;
        } else {
            setJoints(ik_joints);
            QString msg = QString("🔴 碰撞报警: 逆解成功，但当前处于干涉状态 [%1]").arg(coll_details.isEmpty() ? "障碍物" : coll_details);
            emit sigLogMessage(3, msg);
            emit sigCartesianJogFeedback(2, msg);
            return true;
        }
    }

    // 3. 逆解成功且无碰撞
    setJoints(ik_joints);
    QString msg = QString("✔ 步进成功: 逆解正常，位姿安全无干涉");
    emit sigCartesianJogFeedback(0, msg);
    return true;
}

void SimulationController::performCollisionCheck() {
    if (!robot_loaded_) return;

    std::vector<robot_planner::ContactInfo> contacts;
    bool collided = planner_->checkCollisionDetailed(current_joints_, contacts, 0.02);

    std::vector<std::string> colliding_links;
    for (const auto& c : contacts) {
        if (c.distance <= 0.001) { // 处于干涉或危险接触状态
            if (!c.link_name1.empty()) colliding_links.push_back(c.link_name1);
            if (!c.link_name2.empty()) colliding_links.push_back(c.link_name2);
        }
    }

    emit sigCollisionState(collided, colliding_links, contacts);
}

// 障碍物管理
bool SimulationController::addBoxObstacle(const std::string& name, double x, double y, double z, double dx, double dy, double dz) {
    if (!robot_loaded_) return false;
    if (!planner_->addBox(name, x, y, z, dx, dy, dz)) {
        emit sigLogMessage(3, QString("添加立方体障碍物失败: %1").arg(QString::fromStdString(planner_->getLastError())));
        return false;
    }

    ObstacleData data;
    data.name = name;
    data.type = ObstacleType::BOX;
    data.dimensions = {dx, dy, dz};
    data.pose = {x, y, z, 0, 0, 0, 1};
    obstacles_[name] = data;

    emit sigObstaclesChanged();
    performCollisionCheck();
    emit sigLogMessage(1, QString("成功添加立方体障碍物: %1").arg(QString::fromStdString(name)));
    return true;
}

bool SimulationController::addSphereObstacle(const std::string& name, double x, double y, double z, double radius) {
    if (!robot_loaded_) return false;
    if (!planner_->addSphere(name, x, y, z, radius)) {
        emit sigLogMessage(3, QString("添加球体障碍物失败: %1").arg(QString::fromStdString(planner_->getLastError())));
        return false;
    }

    ObstacleData data;
    data.name = name;
    data.type = ObstacleType::SPHERE;
    data.dimensions = {radius};
    data.pose = {x, y, z, 0, 0, 0, 1};
    obstacles_[name] = data;

    emit sigObstaclesChanged();
    performCollisionCheck();
    emit sigLogMessage(1, QString("成功添加球体障碍物: %1").arg(QString::fromStdString(name)));
    return true;
}

bool SimulationController::addCylinderObstacle(const std::string& name, double radius, double length, const std::vector<double>& pose) {
    if (!robot_loaded_) return false;
    if (!planner_->addCylinder(name, radius, length, pose)) {
        emit sigLogMessage(3, QString("添加圆柱障碍物失败: %1").arg(QString::fromStdString(planner_->getLastError())));
        return false;
    }

    ObstacleData data;
    data.name = name;
    data.type = ObstacleType::CYLINDER;
    data.dimensions = {radius, length};
    data.pose = pose;
    obstacles_[name] = data;

    emit sigObstaclesChanged();
    performCollisionCheck();
    emit sigLogMessage(1, QString("成功添加圆柱障碍物: %1").arg(QString::fromStdString(name)));
    return true;
}

bool SimulationController::addMeshObstacle(const std::string& name, const std::string& file_path, const std::vector<double>& pose) {
    if (!robot_loaded_) return false;

    std::vector<double> vertices;
    std::vector<int> faces;
    if (!MeshLoader::loadRawMeshData(file_path, vertices, faces)) {
        emit sigLogMessage(3, QString("解析网格模型数据失败: %1").arg(QString::fromStdString(file_path)));
        return false;
    }

    if (!planner_->addMesh(name, vertices, faces, pose)) {
        emit sigLogMessage(3, QString("添加网格障碍物失败: %1").arg(QString::fromStdString(planner_->getLastError())));
        return false;
    }

    ObstacleData data;
    data.name = name;
    data.type = ObstacleType::MESH;
    data.file_path = file_path;
    data.pose = pose;
    obstacles_[name] = data;

    emit sigObstaclesChanged();
    performCollisionCheck();
    emit sigLogMessage(1, QString("成功导入网格障碍物: %1").arg(QString::fromStdString(name)));
    return true;
}

bool SimulationController::removeObstacle(const std::string& name) {
    if (!robot_loaded_) return false;
    if (planner_->removeObstacle(name)) {
        obstacles_.erase(name);
        emit sigObstaclesChanged();
        performCollisionCheck();
        emit sigLogMessage(0, QString("已移除障碍物: %1").arg(QString::fromStdString(name)));
        return true;
    }
    return false;
}

bool SimulationController::clearAllObstacles() {
    if (!robot_loaded_) return false;
    if (planner_->clearObstacles()) {
        obstacles_.clear();
        emit sigObstaclesChanged();
        performCollisionCheck();
        emit sigLogMessage(0, "已清空场景中所有用户障碍物");
        return true;
    }
    return false;
}

bool SimulationController::attachObstacle(const std::string& name, const std::string& link_name) {
    if (!robot_loaded_) return false;
    if (planner_->attachObject(name, link_name, current_joints_)) {
        if (obstacles_.find(name) != obstacles_.end()) {
            obstacles_[name].is_attached = true;
            obstacles_[name].attached_link = link_name;
        }
        emit sigObstaclesChanged();
        emit sigLogMessage(1, QString("工件 %1 已附着挂载到连杆 %2").arg(QString::fromStdString(name), QString::fromStdString(link_name)));
        return true;
    }
    return false;
}

bool SimulationController::detachObstacle(const std::string& name) {
    if (!robot_loaded_) return false;
    if (planner_->detachObject(name, current_joints_)) {
        if (obstacles_.find(name) != obstacles_.end()) {
            obstacles_[name].is_attached = false;
            obstacles_[name].attached_link.clear();
        }
        emit sigObstaclesChanged();
        emit sigLogMessage(0, QString("工件 %1 已脱离连杆并放回世界坐标系").arg(QString::fromStdString(name)));
        return true;
    }
    return false;
}

// 点位管理
bool SimulationController::saveWaypoint(const std::string& name) {
    if (!robot_loaded_) return false;
    Waypoint wp;
    wp.name = name;
    wp.joint_angles = current_joints_;
    wp.tcp_pose = current_tcp_pose_;
    waypoints_[name] = wp;
    emit sigWaypointsChanged();
    emit sigLogMessage(1, QString("已保存示教点位: %1").arg(QString::fromStdString(name)));
    return true;
}

bool SimulationController::deleteWaypoint(const std::string& name) {
    auto it = waypoints_.find(name);
    if (it != waypoints_.end()) {
        waypoints_.erase(it);
        emit sigWaypointsChanged();
        emit sigLogMessage(0, QString("已删除示教点位: %1").arg(QString::fromStdString(name)));
        return true;
    }
    return false;
}

bool SimulationController::moveToWaypoint(const std::string& name) {
    auto it = waypoints_.find(name);
    if (it != waypoints_.end()) {
        return setJoints(it->second.joint_angles);
    }
    return false;
}

// 轨迹规划
bool SimulationController::planPtp(const std::vector<double>& start_joints, 
                                   const std::vector<double>& target_joints,
                                   double vel_scale, double acc_scale, 
                                   double safety_margin, double timeout) {
    if (!robot_loaded_) return false;
    emit sigLogMessage(0, "正在启动自由空间避障规划 (PTP)...");

    current_trajectory_.clear();
    bool ok = planner_->planFreespace(start_joints, target_joints, current_trajectory_,
                                      vel_scale, acc_scale, timeout, 0.01, safety_margin, 20.0, "PRM");

    if (ok && !current_trajectory_.empty()) {
        double duration = current_trajectory_.time_stamps.back();
        int failed_idx = -1;
        std::string reason;
        bool valid = planner_->validateTrajectory(current_trajectory_, &failed_idx, &reason);

        QString statusText = QString("PTP 规划成功! 路径点: %1, 时长: %2s, 质检: %3")
                                .arg(current_trajectory_.size())
                                .arg(duration, 0, 'f', 2)
                                .arg(valid ? "合规安全" : QString::fromStdString(reason));

        emit sigLogMessage(1, statusText);
        emit sigTrajectoryGenerated(true, static_cast<int>(current_trajectory_.size()), duration, statusText);
        stop();
        return true;
    } else {
        QString err = QString::fromStdString(planner_->getLastError());
        emit sigLogMessage(3, QString("PTP 规划失败: %1").arg(err));
        emit sigTrajectoryGenerated(false, 0, 0.0, err);
        return false;
    }
}

bool SimulationController::planLin(const std::vector<double>& start_joints, 
                                   const std::vector<double>& target_pose,
                                   double vel_scale, double acc_scale,
                                   double step_size, double safety_margin) {
    if (!robot_loaded_) return false;
    emit sigLogMessage(0, "正在启动笛卡尔直线插补规划 (LIN)...");

    current_trajectory_.clear();
    bool ok = planner_->planLinear(start_joints, target_pose, current_trajectory_,
                                   vel_scale, acc_scale, step_size, safety_margin, 20.0);

    if (ok && !current_trajectory_.empty()) {
        double duration = current_trajectory_.time_stamps.back();
        QString statusText = QString("LIN 规划成功! 路径点: %1, 时长: %2s")
                                .arg(current_trajectory_.size())
                                .arg(duration, 0, 'f', 2);

        emit sigLogMessage(1, statusText);
        emit sigTrajectoryGenerated(true, static_cast<int>(current_trajectory_.size()), duration, statusText);
        stop();
        return true;
    } else {
        QString err = QString::fromStdString(planner_->getLastError());
        emit sigLogMessage(3, QString("LIN 直线规划失败: %1").arg(err));
        emit sigTrajectoryGenerated(false, 0, 0.0, err);
        return false;
    }
}

bool SimulationController::planCirc(const std::vector<double>& start_joints,
                                    const std::vector<double>& aux_pose,
                                    const std::vector<double>& target_pose,
                                    double vel_scale, double acc_scale,
                                    double step_size, double safety_margin) {
    if (!robot_loaded_) return false;
    emit sigLogMessage(0, "正在启动笛卡尔圆弧插补规划 (CIRC)...");

    current_trajectory_.clear();
    bool ok = planner_->planCircular(start_joints, aux_pose, target_pose, current_trajectory_,
                                     vel_scale, acc_scale, step_size, safety_margin, 20.0);

    if (ok && !current_trajectory_.empty()) {
        double duration = current_trajectory_.time_stamps.back();
        QString statusText = QString("CIRC 规划成功! 路径点: %1, 时长: %2s")
                                .arg(current_trajectory_.size())
                                .arg(duration, 0, 'f', 2);

        emit sigLogMessage(1, statusText);
        emit sigTrajectoryGenerated(true, static_cast<int>(current_trajectory_.size()), duration, statusText);
        stop();
        return true;
    } else {
        QString err = QString::fromStdString(planner_->getLastError());
        emit sigLogMessage(3, QString("CIRC 圆弧规划失败: %1").arg(err));
        emit sigTrajectoryGenerated(false, 0, 0.0, err);
        return false;
    }
}

// 播放控制
double SimulationController::getTotalTrajectoryDuration() const {
    if (current_trajectory_.empty() || current_trajectory_.time_stamps.empty()) return 0.0;
    return current_trajectory_.time_stamps.back();
}

void SimulationController::play() {
    if (current_trajectory_.empty()) return;
    is_playing_ = true;
    playback_timer_.start(16); // 约 60 FPS
    emit sigPlaybackStateChanged(true);
}

void SimulationController::pause() {
    is_playing_ = false;
    playback_timer_.stop();
    emit sigPlaybackStateChanged(false);
}

void SimulationController::stop() {
    pause();
    seek(0.0);
}

void SimulationController::seek(double time_sec) {
    double total = getTotalTrajectoryDuration();
    current_play_time_ = std::clamp(time_sec, 0.0, total);

    std::vector<double> interpolated_joints;
    interpolateTrajectory(current_play_time_, interpolated_joints);
    if (!interpolated_joints.empty()) {
        setJoints(interpolated_joints);
    }
    emit sigPlaybackTimeChanged(current_play_time_, total);
}

void SimulationController::setPlaybackSpeed(double speed) {
    play_speed_ = std::max(0.1, speed);
}

void SimulationController::setLooping(bool loop) {
    is_looping_ = loop;
}

void SimulationController::onPlaybackTick() {
    double total = getTotalTrajectoryDuration();
    if (total <= 0.0) {
        stop();
        return;
    }

    current_play_time_ += 0.016 * play_speed_;
    if (current_play_time_ >= total) {
        if (is_looping_) {
            current_play_time_ = 0.0;
        } else {
            current_play_time_ = total;
            seek(current_play_time_);
            pause();
            return;
        }
    }
    seek(current_play_time_);
}

void SimulationController::interpolateTrajectory(double time_sec, std::vector<double>& joints_out) {
    if (current_trajectory_.empty() || current_trajectory_.positions.empty()) return;

    const auto& times = current_trajectory_.time_stamps;
    const auto& positions = current_trajectory_.positions;
    size_t n = times.size();

    if (time_sec <= times.front()) {
        joints_out = positions.front();
        return;
    }
    if (time_sec >= times.back()) {
        joints_out = positions.back();
        return;
    }

    // 二分查找时间区间
    auto it = std::lower_bound(times.begin(), times.end(), time_sec);
    size_t idx2 = std::distance(times.begin(), it);
    size_t idx1 = idx2 > 0 ? idx2 - 1 : 0;

    double t1 = times[idx1];
    double t2 = times[idx2];
    double dt = t2 - t1;
    double alpha = dt > 1e-6 ? (time_sec - t1) / dt : 0.0;

    const auto& p1 = positions[idx1];
    const auto& p2 = positions[idx2];
    joints_out.resize(p1.size());
    for (size_t i = 0; i < p1.size(); ++i) {
        joints_out[i] = (1.0 - alpha) * p1[i] + alpha * p2[i];
    }
}

} // namespace sim_app
