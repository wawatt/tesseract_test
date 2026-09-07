#pragma once

#include <QObject>
#include <QTimer>
#include <vector>
#include <string>
#include <map>
#include <memory>

#include "robot_planner/robot_planner.h"

namespace sim_app {

struct Waypoint {
    std::string name;
    std::vector<double> joint_angles;
    std::vector<double> tcp_pose; // [x, y, z, qx, qy, qz, qw]
};

enum class ObstacleType {
    BOX,
    SPHERE,
    CYLINDER,
    CAPSULE,
    MESH,
    POINT_CLOUD
};

struct ObstacleData {
    std::string name;
    ObstacleType type;
    std::vector<double> dimensions; // box: dx,dy,dz; sphere: r; cyl/cap: r,len
    std::vector<double> pose{0, 0, 0, 0, 0, 0, 1};
    std::string file_path;
    bool is_attached{false};
    std::string attached_link;
};

class SimulationController : public QObject {
    Q_OBJECT

public:
    explicit SimulationController(QObject* parent = nullptr);
    virtual ~SimulationController();

    bool initDefaultRobot();
    bool loadRobot(const std::string& urdf_path, 
                   const std::string& srdf_path,
                   const std::string& manip_name,
                   const std::string& base_link,
                   const std::string& tool_link);

    // 状态查询
    bool isRobotLoaded() const { return robot_loaded_; }
    const std::string& getRobotName() const { return robot_name_; }
    const std::vector<double>& getCurrentJoints() const { return current_joints_; }
    const std::vector<double>& getCurrentTcpPose() const { return current_tcp_pose_; }
    const std::vector<double>& getJointLowerLimits() const { return joint_lower_; }
    const std::vector<double>& getJointUpperLimits() const { return joint_upper_; }
    const std::vector<std::string>& getJointNames() const { return joint_names_; }
    const std::vector<std::string>& getLinkNames() const { return link_names_; }
    const std::map<std::string, Waypoint>& getWaypoints() const { return waypoints_; }
    const std::map<std::string, ObstacleData>& getObstacles() const { return obstacles_; }
    const robot_planner::JointTrajectory& getCurrentTrajectory() const { return current_trajectory_; }
    bool isPlaying() const { return is_playing_; }
    double getCurrentPlaybackTime() const { return current_play_time_; }
    double getTotalTrajectoryDuration() const;

    // 各 Link 世界位姿获取 (供 OSG 渲染更新)
    bool getLinkWorldPoses(std::vector<std::string>& names_out, 
                           std::vector<std::vector<double>>& poses_out);

    // 示教控制
    bool setJoints(const std::vector<double>& joints);
    bool jogJoint(size_t joint_idx, double delta_rad);
    bool jogCartesian(int axis_idx, double delta_m_or_rad, bool stop_on_collision = true);

    // 障碍物管理
    bool addBoxObstacle(const std::string& name, double x, double y, double z, double dx, double dy, double dz);
    bool addSphereObstacle(const std::string& name, double x, double y, double z, double radius);
    bool addCylinderObstacle(const std::string& name, double radius, double length, const std::vector<double>& pose);
    bool addMeshObstacle(const std::string& name, const std::string& file_path, const std::vector<double>& pose);
    bool removeObstacle(const std::string& name);
    bool clearAllObstacles();
    bool attachObstacle(const std::string& name, const std::string& link_name = "tool0");
    bool detachObstacle(const std::string& name);

    // 点位管理
    bool saveWaypoint(const std::string& name);
    bool deleteWaypoint(const std::string& name);
    bool moveToWaypoint(const std::string& name);

    // 轨迹规划
    bool planPtp(const std::vector<double>& start_joints, 
                 const std::vector<double>& target_joints,
                 double vel_scale = 1.0, double acc_scale = 1.0, 
                 double safety_margin = 0.025, double timeout = 10.0);

    bool planLin(const std::vector<double>& start_joints, 
                 const std::vector<double>& target_pose,
                 double vel_scale = 1.0, double acc_scale = 1.0,
                 double step_size = 0.02, double safety_margin = 0.025);

    bool planCirc(const std::vector<double>& start_joints,
                  const std::vector<double>& aux_pose,
                  const std::vector<double>& target_pose,
                  double vel_scale = 1.0, double acc_scale = 1.0,
                  double step_size = 0.02, double safety_margin = 0.025);

    // 轨迹播放控制
    void play();
    void pause();
    void stop();
    void seek(double time_sec);
    void setPlaybackSpeed(double speed);
    void setLooping(bool loop);

signals:
    void sigRobotLoaded(bool success, const QString& robot_name);
    void sigJointsUpdated(const std::vector<double>& joints, const std::vector<double>& tcp_pose);
    void sigCollisionState(bool in_collision, 
                           const std::vector<std::string>& colliding_links, 
                           const std::vector<robot_planner::ContactInfo>& contacts);
    void sigObstaclesChanged();
    void sigWaypointsChanged();
    void sigTrajectoryGenerated(bool success, int num_points, double duration, const QString& msg);
    void sigPlaybackTimeChanged(double current_time, double total_duration);
    void sigPlaybackStateChanged(bool is_playing);
    void sigLogMessage(int level, const QString& text); // 0=Info, 1=Success, 2=Warn, 3=Error
    void sigCartesianJogFeedback(int status_code, const QString& message); // 0=Success, 1=IK_Failed, 2=Collision

private slots:
    void onPlaybackTick();

private:
    void performCollisionCheck();
    void interpolateTrajectory(double time_sec, std::vector<double>& joints_out);

    std::unique_ptr<robot_planner::RobotPlanner> planner_;
    bool robot_loaded_{false};
    std::string robot_name_;
    std::string base_link_{"base_link"};
    std::string tool_link_{"tool0"};

    std::vector<std::string> joint_names_;
    std::vector<std::string> link_names_;
    std::vector<double> joint_lower_;
    std::vector<double> joint_upper_;
    std::vector<double> current_joints_;
    std::vector<double> current_tcp_pose_;

    std::map<std::string, ObstacleData> obstacles_;
    std::map<std::string, Waypoint> waypoints_;

    robot_planner::JointTrajectory current_trajectory_;
    QTimer playback_timer_;
    bool is_playing_{false};
    double current_play_time_{0.0};
    double play_speed_{1.0};
    bool is_looping_{false};
};

} // namespace sim_app
