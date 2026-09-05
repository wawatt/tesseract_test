#pragma once

#include <vector>
#include <string>
#include <memory>

namespace robot_planner {

/**
 * @brief 规划器底层后端类型
 */
enum class PlannerBackend {
    TESSERACT = 0,  // 原生 Tesseract + Bullet/FCL 引擎
    VAMP = 1,       // 纯 VAMP AVX2 SIMD 硬件加速 (所有障碍物自动转 Octree/AABB)
    AUTO = 2        // 自动探测 (若是 r2000ic_165f 且动态库存在则使用 VAMP，否则使用 Tesseract)
};

/**
 * @brief 规划器状态与错误诊断码
 */
enum class PlannerStatus {
    SUCCESS = 0,                    // 规划成功
    NOT_INITIALIZED = 1,            // 规划器未初始化
    INVALID_ARGUMENTS = 2,          // 输入参数错误
    IK_FAILED = 3,                  // 逆运动学无可行解
    JOINT_LIMIT_VIOLATED = 4,       // 超出 URDF 关节物理软/硬限位
    SINGULARITY_DETECTED = 5,       // 检测到手腕或连杆奇异点 / 轴剧烈跃度
    COLLISION_DETECTED = 6,         // 发生碰撞
    PLANNING_TIMEOUT = 7,           // 全局避障规划超时
    TRAJECTORY_FAILED = 8,          // 轨迹平滑或动力学时间参数化失败
    OBSTACLE_NOT_FOUND = 9,         // 目标障碍物不存在
    INTERNAL_ERROR = 10             // 底层引擎未知异常
};

/**
 * @brief 统一关节轨迹结构体 (仅由基础标准类型构成)
 */
struct JointTrajectory {
    std::vector<std::vector<double>> positions;     // 路径点关节位置 [rad]
    std::vector<std::vector<double>> velocities;    // 路径点关节速度 [rad/s]
    std::vector<std::vector<double>> accelerations; // 路径点关节加速度 [rad/s^2]
    std::vector<double> time_stamps;               // 相对于起点的时间戳 [s]

    void clear() {
        positions.clear();
        velocities.clear();
        accelerations.clear();
        time_stamps.clear();
    }

    bool empty() const {
        return positions.empty();
    }

    size_t size() const {
        return positions.size();
    }
};

/**
 * @brief 碰撞与接触几何信息结构体 (仅由标准基础类型构成)
 */
struct ContactInfo {
    std::string link_name1;         // 发生接触的连杆/物体1
    std::string link_name2;         // 发生接触的连杆/物体2
    double distance{0.0};           // 距离 (负值表示侵入深度，正值表示净空安全裕度) [m]
    std::vector<double> point1;     // 物体1上的接触点世界坐标 [x, y, z]
    std::vector<double> point2;     // 物体2上的接触点世界坐标 [x, y, z]
    std::vector<double> normal;     // 接触法向量 [nx, ny, nz] (指向物体2)
};

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
     * @param backend 规划后端类型 (TESSERACT / VAMP / AUTO，默认 AUTO)
     * @param custom_plugin_path 自定义加速动态库路径
     * @return 成功返回 true
     */
    bool init(const std::string& urdf_path, const std::string& srdf_path, 
              const std::string& manipulator_name, 
              const std::string& base_link, 
              const std::string& tool_link,
              PlannerBackend backend = PlannerBackend::AUTO,
              const std::string& custom_plugin_path = "");

    /**
     * @brief 获取当前生效的规划器后端类型
     */
    PlannerBackend getBackend() const;

    /**
     * @brief 获取最近一次调用的错误状态码
     */
    PlannerStatus getLastErrorStatus() const;

    /**
     * @brief 获取最近一次调用的可读错误详情说明
     */
    std::string getLastError() const;

    // ---------------------------------------------------------
    // 运动学与碰撞检测 (Kinematics & Collision)
    // ---------------------------------------------------------

    /**
     * @brief 正向运动学 (FK, 计算默认 tool_link 末端位姿)
     * @param joint_angles 关节角度向量 [rad]
     * @param pose_out 输出末端位姿 [x, y, z, qx, qy, qz, qw]
     * @return 成功返回 true
     */
    bool computeFK(const std::vector<double>& joint_angles, std::vector<double>& pose_out);

    /**
     * @brief 正向运动学 (FK, 计算任意指定连杆的世界位姿)
     * @param joint_angles 关节角度向量 [rad]
     * @param link_name 目标连杆名称 (如 "link_3", "link_5", "tool0")
     * @param pose_out 输出指定连杆位姿 [x, y, z, qx, qy, qz, qw]
     * @return 成功返回 true
     */
    bool computeFKForLink(const std::vector<double>& joint_angles, const std::string& link_name, std::vector<double>& pose_out);

    /**
     * @brief 逆向运动学 (IK, 自动过滤 URDF 物理限位，并优先返回无碰撞的最小位移解)
     * @param pose 目标位姿 [x, y, z, qx, qy, qz, qw]
     * @param seed_joint_angles 初始参考关节角度 (用于择优最小位移解)
     * @param joint_angles_out 输出求解的关节角度
     * @return 成功返回 true
     */
    bool computeIK(const std::vector<double>& pose, const std::vector<double>& seed_joint_angles, std::vector<double>& joint_angles_out);

    /**
     * @brief 逆向运动学全部解析解 (自动过滤超出 URDF 关节限位的解)
     * @param pose 目标位姿 [x, y, z, qx, qy, qz, qw]
     * @param all_solutions_out 输出所有符合关节物理限位的解析解集合
     * @return 至少有一个合法解返回 true
     */
    bool computeAllIK(const std::vector<double>& pose, std::vector<std::vector<double>>& all_solutions_out);

    /**
     * @brief 无碰撞逆向运动学 (同时过滤 URDF 关节限位与全场景障碍物干涉)
     * @param pose 目标位姿 [x, y, z, qx, qy, qz, qw]
     * @param seed_joint_angles 初始参考关节角度 (用于择优最小位移解)
     * @param joint_angles_out 输出求解的无碰撞关节角度
     * @return 找到合法无碰撞解返回 true
     */
    bool computeCollisionFreeIK(const std::vector<double>& pose, const std::vector<double>& seed_joint_angles, std::vector<double>& joint_angles_out);

    /**
     * @brief 计算所有合法且安全无碰撞的解析逆解集合 (cuRobo 风格多种子解生成器)
     * @param pose 目标位姿 [x, y, z, qx, qy, qz, qw]
     * @param all_solutions_out 输出所有符合关节限位且与当前场景无碰撞的解集合 (最多 8 组)
     * @return 至少存在一个无碰撞解返回 true
     */
    bool computeAllCollisionFreeIK(const std::vector<double>& pose, std::vector<std::vector<double>>& all_solutions_out);

    /**
     * @brief 计算所有合法且安全无碰撞的解析逆解集合，并按与参考种子角度的距离升序排列
     * @param pose 目标位姿 [x, y, z, qx, qy, qz, qw]
     * @param seed_joint_angles 参考种子关节角度
     * @param all_solutions_out 输出按接近参考点排序的无碰撞解集合
     * @return 至少存在一个无碰撞解返回 true
     */
    bool computeAllCollisionFreeIK(const std::vector<double>& pose, const std::vector<double>& seed_joint_angles, std::vector<std::vector<double>>& all_solutions_out);

    /**
     * @brief 计算指定关节状态下末端或指定连杆的 6xN 几何雅可比矩阵
     * @param joint_angles 关节角度 [rad]
     * @param jacobian_out 输出扁平数组 (大小 6 * N，行优先排列)
     * @param link_name 目标连杆名称 (传空字符串则默认为 tool_link)
     * @return 成功返回 true
     */
    bool calcJacobian(const std::vector<double>& joint_angles, std::vector<double>& jacobian_out, const std::string& link_name = "");

    /**
     * @brief 计算吉川可操作度指标 (Yoshikawa Manipulability Index)
     * @details 标量度量 w = sqrt(det(J * J^T))。接近 0 说明处于或临近奇异构型。
     * @param joint_angles 关节角度 [rad]
     * @param score_out 输出可操作度指标
     * @return 成功返回 true
     */
    bool computeManipulability(const std::vector<double>& joint_angles, double& score_out);

    /**
     * @brief 关节状态碰撞检测 (发生碰撞时 getLastError 会包含具体冲突连杆信息)
     * @param joint_angles 需要检测的关节状态
     * @return 发生碰撞返回 true，安全无碰撞返回 false
     */
    bool checkCollision(const std::vector<double>& joint_angles);

    /**
     * @brief 查询指定关节状态下的所有碰撞/接触对详细几何信息 (带接触点与法向量)
     * @param joint_angles 关节角度 [rad]
     * @param contacts_out 输出的所有接触/碰撞详细信息
     * @param contact_distance 接触判定阈值 (米，<=0 仅返回发生干涉的对，>0 返回小于该安全净距的近邻对)
     * @return 存在干涉或小于安全裕度时返回 true
     */
    bool checkCollisionDetailed(const std::vector<double>& joint_angles, 
                                std::vector<ContactInfo>& contacts_out, 
                                double contact_distance = 0.0);

    /**
     * @brief 独立轨迹全量合规性质检器 (物理软硬限位、速度超限、加速度超限、时间戳单调性、全轨碰撞)
     * @param trajectory 需要质检的完整轨迹 (在 VAMP 模式下利用 AVX2 SIMD 微秒级并行质检)
     * @param failed_waypoint_index [可选输出] 首个违规的路径点序号 (0-indexed)
     * @param reason [可选输出] 违规的具体详细说明
     * @return 全部合规返回 true，任一条件违规返回 false
     */
    bool validateTrajectory(const JointTrajectory& trajectory, 
                            int* failed_waypoint_index = nullptr, 
                            std::string* reason = nullptr);

    // ---------------------------------------------------------
    // 场景与障碍物全生命周期管理 (Scene & Obstacles)
    // ---------------------------------------------------------

    /**
     * @brief 在场景中添加立方体 (Box) 障碍物
     * @param name 障碍物唯一名称
     * @param x, y, z 中心位置 (米)
     * @param dim_x, dim_y, dim_z 长宽高尺寸 (米)
     * @return 成功返回 true
     */
    bool addBox(const std::string& name, double x, double y, double z, double dim_x, double dim_y, double dim_z);

    /**
     * @brief 在场景中添加球体 (Sphere) 障碍物
     * @param name 障碍物唯一名称
     * @param x, y, z 球心位置 (米)
     * @param radius 球半径 (米)
     * @return 成功返回 true
     */
    bool addSphere(const std::string& name, double x, double y, double z, double radius);

    /**
     * @brief 在场景中添加圆柱体 (Cylinder) 障碍物
     * @param name 障碍物唯一名称
     * @param radius 圆柱半径 (米)
     * @param length 圆柱长度/高度 (米)
     * @param pose 位姿 [x, y, z, qx, qy, qz, qw]
     * @return 成功返回 true
     */
    bool addCylinder(const std::string& name, double radius, double length, 
                    const std::vector<double>& pose = {0,0,0, 0,0,0,1});

    /**
     * @brief 在场景中添加胶囊体 (Capsule) 障碍物
     * @param name 障碍物唯一名称
     * @param radius 胶囊体两端半球及圆柱半径 (米)
     * @param length 胶囊体圆柱段长度 (米)
     * @param pose 位姿 [x, y, z, qx, qy, qz, qw]
     * @return 成功返回 true
     */
    bool addCapsule(const std::string& name, double radius, double length, 
                   const std::vector<double>& pose = {0,0,0, 0,0,0,1});

    /**
     * @brief 在场景中添加网格模型 (Mesh) 障碍物
     * @param name 障碍物唯一名称
     * @param vertices 顶点坐标扁平数组 [x1, y1, z1, x2, y2, z2, ...]
     * @param faces 三角形顶点索引扁平数组 [v1, v2, v3, ...]
     * @param pose 位姿 [x, y, z, qx, qy, qz, qw]
     * @return 成功返回 true
     */
    bool addMesh(const std::string& name,
                 const std::vector<double>& vertices,
                 const std::vector<int>& faces,
                 const std::vector<double>& pose = {0,0,0, 0,0,0,1});

    /**
     * @brief 在场景中添加点云 (PointCloud) 障碍物
     * @param name 障碍物唯一名称
     * @param points 点云坐标扁平数组 [x1, y1, z1, x2, y2, z2, ...]
     * @param resolution 八叉树空间体素分辨率 (米)
     * @param pose 位姿 [x, y, z, qx, qy, qz, qw]
     * @return 成功返回 true
     */
    bool addPointCloud(const std::string& name,
                       const std::vector<double>& points,
                       double resolution = 0.01,
                       const std::vector<double>& pose = {0,0,0, 0,0,0,1});

    /**
     * @brief 设置两个连杆/障碍物之间是否允许碰撞 (允许碰撞矩阵 ACM 白名单/黑名单)
     * @param link1 连杆或障碍物名称
     * @param link2 连杆或障碍物名称
     * @param allowed true 为允许碰触 (忽略干涉)，false 为恢复常规干涉检测
     * @return 成功返回 true
     */
    bool setAllowedCollision(const std::string& link1, const std::string& link2, bool allowed);

    /**
     * @brief 查询两个连杆/障碍物之间是否处于允许碰撞白名单中
     */
    bool isCollisionAllowed(const std::string& link1, const std::string& link2) const;

    /**
     * @brief 移除场景中的障碍物
     * @param name 障碍物唯一名称
     * @return 成功返回 true
     */
    bool removeObstacle(const std::string& name);

    /**
     * @brief 清空场景中所有用户添加的障碍物
     * @return 成功返回 true
     */
    bool clearObstacles();

    /**
     * @brief 获取场景中当前已添加的所有障碍物名称列表
     */
    std::vector<std::string> getObstacleNames() const;

    /**
     * @brief 判断指定名称的障碍物是否存在
     */
    bool hasObstacle(const std::string& name) const;

    /**
     * @brief 将场景中已有的障碍物挂载/附着到机械臂指定连杆上 (用于末端工件抓取随动碰撞检测)
     * @param obstacle_name 障碍物名称
     * @param link_name 挂载的目标连杆 (传空字符串则默认挂载到 tool_link)
     * @param current_joints [可选] 抓取时的关节角度 (用于精确计算相对挂载矩阵，传空则自动使用当前有效位形)
     * @return 成功返回 true
     */
    bool attachObject(const std::string& obstacle_name, 
                      const std::string& link_name = "", 
                      const std::vector<double>& current_joints = {});

    /**
     * @brief 将已附着到机械臂上的物体分离/解除挂载，放回世界坐标系
     * @param obstacle_name 障碍物名称
     * @param current_joints [可选] 分离时的关节角度 (用于更新物体放置位姿，传空则自动使用当前有效位形)
     * @return 成功返回 true
     */
    bool detachObject(const std::string& obstacle_name, 
                      const std::vector<double>& current_joints = {});

    // ---------------------------------------------------------
    // 运动规划接口 (Motion Planning with Trajectory Dynamics)
    // ---------------------------------------------------------

    /**
     * @brief 自由空间点到点避障规划 (全动力学轨迹输出)
     * @param start_joints 起始关节角度 [rad]
     * @param target_joints 目标关节角度 [rad]
     * @param trajectory_out 输出完整时间参数化轨迹 (含位置、速度、加速度、时间戳)
     * @param max_velocity_scaling 最大速度缩放比例 (0.01 ~ 1.0，默认 1.0)
     * @param max_acceleration_scaling 最大加速度缩放比例 (0.01 ~ 1.0，默认 1.0)
     * @param planning_time 全局寻路最大超时时间 (秒，默认 10.0)
     * @param range 采样步长 (弧度，默认 0.01)
     * @param safety_margin 避障安全裕度 (米，默认 0.025)
     * @param collision_coeff 碰撞代价系数 (默认 20.0)
    /**
     * @brief 预先构建/预热 PRM 稠密路标图 (cuRobo PRMGraphPlanner 风格)
     * @details 在静态工位启动时预先对工作空间进行采样构图，后续在线查询只需毫秒级图搜索 + TrajOpt，替代耗时数秒的重复树搜索
     * @param warmup_time 预热采样时长 (秒，默认 0.3)
     * @return 成功返回 true
     */
    bool warmupRoadmap(double warmup_time = 0.3);

    /**
     * @brief 自由空间点到点避障规划 (全动力学轨迹输出)
     * @param start_joints 起始关节角度 [rad]
     * @param target_joints 目标关节角度 [rad]
     * @param trajectory_out 输出完整时间参数化轨迹 (含位置、速度、加速度、时间戳)
     * @param max_velocity_scaling 最大速度缩放比例 (0.01 ~ 1.0，默认 1.0)
     * @param max_acceleration_scaling 最大加速度缩放比例 (0.01 ~ 1.0，默认 1.0)
     * @param planning_time 全局寻路最大超时时间 (秒，默认 10.0)
     * @param range 采样步长 (弧度，默认 0.01)
     * @param safety_margin 避障安全裕度 (米，默认 0.025)
     * @param collision_coeff 碰撞代价系数 (默认 20.0)
     * @param planner_type OMPL规划器算法 (默认 "PRM"，支持 "PRM" / "RRTConnect" / "RRTstar")
     * @return 成功返回 true
     */
    bool planFreespace(const std::vector<double>& start_joints, 
                       const std::vector<double>& target_joints, 
                       JointTrajectory& trajectory_out,
                       double max_velocity_scaling = 1.0,
                       double max_acceleration_scaling = 1.0,
                       double planning_time = 10.0,
                       double range = 0.01,
                       double safety_margin = 0.025,
                       double collision_coeff = 20.0,
                       const std::string& planner_type = "PRM");

    /**
     * @brief 笛卡尔末端位姿目标自由空间避障规划 (cuRobo 风格多种子 TrajOpt 轮询寻优)
     * @details 解析目标位姿的所有 OPW 无碰撞候选解 (最多 8 组)，按离起始位形近邻排序依次尝试优化，规避单一解陷入局部坏盆地 (bad basin)
     * @param start_joints 起始关节角度 [rad]
     * @param target_pose 目标位姿 [x, y, z, qx, qy, qz, qw]
     * @param trajectory_out 输出完整时间参数化轨迹
     * @param max_velocity_scaling 最大速度缩放比例 (0.01 ~ 1.0，默认 1.0)
     * @param max_acceleration_scaling 最大加速度缩放比例 (0.01 ~ 1.0，默认 1.0)
     * @param planning_time 全局寻路最大超时时间 (秒，默认 10.0)
     * @param range 采样步长 (弧度，默认 0.01)
     * @param safety_margin 避障安全裕度 (米，默认 0.025)
     * @param collision_coeff 碰撞代价系数 (默认 20.0)
     * @param planner_type OMPL规划器算法 (默认 "PRM"，支持 "PRM" / "RRTConnect" / "RRTstar")
     * @param max_seeds 最多尝试的无碰撞种子数量 (默认 8)
     * @return 成功返回 true
     */
    bool planFreespacePose(const std::vector<double>& start_joints, 
                           const std::vector<double>& target_pose, 
                           JointTrajectory& trajectory_out,
                           double max_velocity_scaling = 1.0,
                           double max_acceleration_scaling = 1.0,
                           double planning_time = 10.0,
                           double range = 0.01,
                           double safety_margin = 0.025,
                           double collision_coeff = 20.0,
                           const std::string& planner_type = "PRM",
                           size_t max_seeds = 8);

    /**
     * @brief 笛卡尔直线规划 (线性插补，带奇异点检测与动力学时间参数化)
     * @param start_joints 起始关节角度 [rad]
     * @param target_pose 目标位姿 [x, y, z, qx, qy, qz, qw]
     * @param trajectory_out 输出完整时间参数化轨迹
     * @param max_velocity_scaling 最大速度缩放比例 (0.01 ~ 1.0，默认 1.0)
     * @param max_acceleration_scaling 最大加速度缩放比例 (0.01 ~ 1.0，默认 1.0)
     * @param step_size 笛卡尔空间位置插补步长 (米，默认 0.02)
     * @param safety_margin 避障安全裕度 (米，默认 0.025)
     * @param collision_coeff 碰撞代价系数 (默认 20.0)
     * @return 成功返回 true
     */
    bool planLinear(const std::vector<double>& start_joints, 
                    const std::vector<double>& target_pose, 
                    JointTrajectory& trajectory_out,
                    double max_velocity_scaling = 1.0,
                    double max_acceleration_scaling = 1.0,
                    double step_size = 0.02,
                    double safety_margin = 0.025,
                    double collision_coeff = 20.0);

    /**
     * @brief 笛卡尔圆弧规划 (圆弧三点插补，带共线/奇异点检测与动力学时间参数化)
     * @param start_joints 起始关节角度 [rad]
     * @param aux_pose 圆弧中间辅助点位姿 [x, y, z, qx, qy, qz, qw]
     * @param target_pose 目标终点位姿 [x, y, z, qx, qy, qz, qw]
     * @param trajectory_out 输出完整时间参数化轨迹
     * @param max_velocity_scaling 最大速度缩放比例 (0.01 ~ 1.0，默认 1.0)
     * @param max_acceleration_scaling 最大加速度缩放比例 (0.01 ~ 1.0，默认 1.0)
     * @param step_size 笛卡尔空间圆弧插补步长 (米，默认 0.02)
     * @param safety_margin 避障安全裕度 (米，默认 0.025)
     * @param collision_coeff 碰撞代价系数 (默认 20.0)
     * @return 成功返回 true
     */
    bool planCircular(const std::vector<double>& start_joints, 
                      const std::vector<double>& aux_pose, 
                      const std::vector<double>& target_pose, 
                      JointTrajectory& trajectory_out,
                      double max_velocity_scaling = 1.0,
                      double max_acceleration_scaling = 1.0,
                      double step_size = 0.02,
                      double safety_margin = 0.025,
                      double collision_coeff = 20.0);

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace robot_planner
