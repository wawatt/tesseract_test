#pragma once

#include <vector>
#include <string>
#include <memory>

namespace robot_planner {

class RobotPlanner {
public:
    RobotPlanner();
    ~RobotPlanner();

    /**
     * @brief 初始化规划器
     * @param urdf_path URDF 模型文件路径
     * @param srdf_path SRDF 配置文件路径
     * @param manipulator_name 规划组名称 (例如: "manipulator")
     * @param base_link 基座连杆名称 (例如: "base_link")
     * @param tool_link 工具连杆名称 (例如: "tool0")
     * @return 成功返回 true
     */
    bool init(const std::string& urdf_path, const std::string& srdf_path, 
              const std::string& manipulator_name, 
              const std::string& base_link, 
              const std::string& tool_link);

    /**
     * @brief 正向运动学 (FK)
     * @param joint_angles 关节角度向量
     * @param pose_out 输出位姿 [x, y, z, qx, qy, qz, qw]
     * @return 成功返回 true
     */
    bool computeFK(const std::vector<double>& joint_angles, std::vector<double>& pose_out);

    /**
     * @brief 逆向运动学 (IK)
     * @param pose 目标位姿 [x, y, z, qx, qy, qz, qw]
     * @param seed_joint_angles 初始猜测关节角度
     * @param joint_angles_out 输出求解的关节角度
     * @return 成功返回 true
     */
    bool computeIK(const std::vector<double>& pose, const std::vector<double>& seed_joint_angles, std::vector<double>& joint_angles_out);

    /**
     * @brief 碰撞检测
     * @param joint_angles 需要检测的关节状态
     * @return 发生碰撞返回 true，安全返回 false
     */
    bool checkCollision(const std::vector<double>& joint_angles);

    // ---------------------------------------------------------
    // 障碍物管理
    // ---------------------------------------------------------

    /**
     * @brief 在场景中添加 Box 障碍物
     * @param name 障碍物唯一名称
     * @param x, y, z 中心位置
     * @param dim_x, dim_y, dim_z 长宽高
     * @return 成功返回 true
     */
    bool addBox(const std::string& name, double x, double y, double z, double dim_x, double dim_y, double dim_z);

    // 添加网格模型 (Mesh) 障碍物
    // vertices: 平铺的顶点坐标 [x1, y1, z1, x2, y2, z2, ...]
    // faces: 平铺的三角形面片顶点索引 [v1, v2, v3, v4, v5, v6, ...] (每三个索引构成一个三角形)
    // pose: [x, y, z, qx, qy, qz, qw]
    bool addMesh(const std::string& name,
                 const std::vector<double>& vertices,
                 const std::vector<int>& faces,
                 const std::vector<double>& pose = {0,0,0, 0,0,0,1});

    // 添加点云 (PointCloud/Octree) 障碍物
    // points: 平铺的点云坐标 [x1, y1, z1, x2, y2, z2, ...]
    // resolution: 八叉树的分辨率 (单位: 米)
    // pose: [x, y, z, qx, qy, qz, qw]
    bool addPointCloud(const std::string& name,
                       const std::vector<double>& points,
                       double resolution = 0.01,
                       const std::vector<double>& pose = {0,0,0, 0,0,0,1});

    /**
     * @brief 移除场景中的障碍物
     * @param name 障碍物唯一名称
     * @return 成功返回 true
     */
    bool removeObstacle(const std::string& name);

    /**
     * @brief Freespace 规划 (自由空间点到点规划，包含避障)
     * @param start_joints 起始关节角度
     * @param target_joints 目标关节角度
     * @param trajectory_out 输出轨迹
     * @param planning_time 全局寻路最大超时时间 (秒，默认 10.0)
     * @param range 采样步长 (弧度，默认 0.01)
     * @param safety_margin 避障安全裕度 (米，默认 0.025)
     * @param collision_coeff 碰撞代价系数 (默认 20.0)
     * @param planner_type OMPL规划器算法类型 ("RRTConnect"/"RRTstar"/"PRM"，默认 "RRTConnect")
     * @return 成功返回 true
     */
    bool planFreespace(const std::vector<double>& start_joints, 
                       const std::vector<double>& target_joints, 
                       std::vector<std::vector<double>>& trajectory_out,
                       double planning_time = 10.0,
                       double range = 0.01,
                       double safety_margin = 0.025,
                       double collision_coeff = 20.0,
                       const std::string& planner_type = "RRTConnect");

    /**
     * @brief Linear 规划 (线性规划，笛卡尔空间直线)
     * @param start_joints 起始关节角度
     * @param target_pose 目标位姿 [x, y, z, qx, qy, qz, qw]
     * @param trajectory_out 输出轨迹
     * @param safety_margin 避障安全裕度 (米，默认 0.025)
     * @param collision_coeff 碰撞代价系数 (默认 20.0)
     * @return 成功返回 true
     */
    bool planLinear(const std::vector<double>& start_joints, 
                    const std::vector<double>& target_pose, 
                    std::vector<std::vector<double>>& trajectory_out,
                    double safety_margin = 0.025,
                    double collision_coeff = 20.0);

    /**
     * @brief Circular 规划 (圆弧规划，笛卡尔空间圆弧)
     * @param start_joints 起始关节角度
     * @param aux_pose 圆弧中间辅助点位姿 [x, y, z, qx, qy, qz, qw]
     * @param target_pose 目标点位姿 [x, y, z, qx, qy, qz, qw]
     * @param trajectory_out 输出轨迹
     * @param safety_margin 避障安全裕度 (米，默认 0.025)
     * @param collision_coeff 碰撞代价系数 (默认 20.0)
     * @return 成功返回 true
     */
    bool planCircular(const std::vector<double>& start_joints, 
                      const std::vector<double>& aux_pose, 
                      const std::vector<double>& target_pose, 
                      std::vector<std::vector<double>>& trajectory_out,
                      double safety_margin = 0.025,
                      double collision_coeff = 20.0);

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace robot_planner
