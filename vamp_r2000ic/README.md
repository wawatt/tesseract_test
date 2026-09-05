# Fanuc R-2000iC/165F VAMP SIMD 向量化运动规划加速模块

本目录是一个**自包含、模块化**的机械臂运动规划硬件加速工程，针对工业级重型机械臂 **Fanuc R-2000iC/165F**，实现了基于 CPU **SIMD（AVX2/AVX-512）** 指令集的超高吞吐量正向运动学（FK）与球体碰撞检测，并深度桥接 **OMPL** 规划库，使全局自由空间避障规划时间从常规的 **数百毫秒级** 缩短至 **1 ~ 5 毫秒级**。

---

## 目录索引

- [1. 背景与技术痛点](#1-背景与技术痛点)
- [2. VAMP 核心原理与数学基础](#2-vamp-核心原理与数学基础)
- [3. Fanuc R-2000iC/165F 落地实现全流程](#3-fanuc-r-2000ic165f-落地实现全流程)
  - [步骤一：URDF 宏展开与几何提取](#步骤一urdf-宏展开与几何提取)
  - [步骤二：模型球体化逼近 (Spherical Decomposition)](#步骤二模型球体化逼近-spherical-decomposition)
  - [步骤三：自碰撞豁免矩阵 (ACM) 配置](#步骤三自碰撞豁免矩阵-acm-配置)
  - [步骤四：SIMD 向量化正运动学与碰撞内核](#步骤四simd-向量化正运动学与碰撞内核)
  - [步骤五：OMPL 适配桥接与高层规划器](#步骤五ompl-适配桥接与高层规划器)
- [4. 如何在 RobotPlanner 中无缝接入](#4-如何在-robotplanner-中无缝接入)
- [5. 编译与基准性能测试](#5-编译与基准性能测试)
- [6. 性能与优势对比](#6-性能与优势对比)

---

## 1. 背景与技术痛点

在传统的基于采样的机械臂运动规划（如 OMPL 的 RRTConnect、RRT*、PRM）中：
1. **碰撞检测是最大瓶颈**：算法需要对数千乃至数万个中间关节状态进行有效性验证（Validity Checking），通常 **85% ~ 95% 的 CPU 时间消耗在碰撞检测上**。
2. **复杂网格计算代价高**：工业机器人的 URDF 通常包含高精度的三角面片网格（STL/DAE，单个连杆可达数万面片）。基于 Bullet 或 FCL 的 GJK/EPA 碰撞算法包含大量的分支跳转、动态内存遍历和浮点除法，无法充分利用现代 CPU 强大的向量并行单元。
3. **无法开箱即用 SIMD**：三角网格形状各异，不能直接装载进 AVX 寄存器（一次操作 8 个 float）。因此，必须对碰撞几何与运动学架构进行根本性的重构。

---

## 2. VAMP 核心原理与数学基础

**VAMP (Vector-Accelerated Motion Planning)** 由 Rice 大学 Kavraki 实验室提出，其核心思想包含三个层面：

### 2.1 空间球体化分解 (Spherical Decomposition)
将每个机械臂连杆复杂的外部多边形网格用一组互相重叠的**外包络球（Bounding Spheres）**进行几何逼近。
- 球体碰撞判定是所有几何体中最简单的：对于球心 $C_1, C_2$ 和半径 $r_1, r_2$，仅需判断：
  $$\|C_1 - C_2\|^2 \le (r_1 + r_2 + \text{margin})^2$$
- 全程只需乘加运算（FMA），无三角函数、无分支开销。

### 2.2 向量化并行（SIMD / AVX2 / AVX-512）
在 AVX2 下，一个 256 位 YMM 寄存器可同时容纳 8 个单精度浮点数。
- 对环境中的障碍物（如 AABB 盒子、地面平面、圆柱）：
  将球心坐标批量载入 SIMD 寄存器，使用 `_mm256_max_ps`、`_mm256_min_ps`、`_mm256_fmadd_ps` 在 **几个时钟周期内同时判定 8 个球体** 与障碍物的相对距离。

### 2.3 提前编译展开 (Ahead-Of-Time Tracing)
消除递归的 `calcFwdKin` 树遍历，针对该机器人固定的 6 轴运动学链，直接手写或代码生成展开的变换矩阵乘法，一次性解算出机械臂全身上下所有球心在世界坐标系下的绝对坐标。

---

## 3. Fanuc R-2000iC/165F 落地实现全流程

```text
[r2000ic_165f.urdf.xacro]
       │
       ▼  (步骤一: xacro 展开)
[r2000ic_165f.urdf]
       │
       ▼  (步骤二: foam/启发式多球体逼近)
[r2000ic_165f_spherized.urdf] ── (步骤三: ACM 豁免) ──> [r2000ic_165f.srdf]
       │
       ▼  (步骤四: SIMD 向量化展开)
[simd_defs.hpp / r2000ic_simd_fk.hpp / r2000ic_collision.hpp]
       │
       ▼  (步骤五: OMPL C++ 适配桥接)
[vamp_ompl_adapter.hpp / vamp_r2000ic_planner.hpp]
```

### 步骤一：URDF 宏展开与几何提取
Fanuc 官方模型位于 `asset/fanuc_r2000_description/robot/r2000ic_165f.urdf.xacro`。
使用本模块提供的脚本将其展开并生成标准 URDF：
```bash
python scripts/export_urdf.py
```
展开后的完整标准模型保存于 `models/r2000ic_165f.urdf`。
所依赖的 7 个连杆的 visual/collision 3D 网格模型已直接收录在 `models/meshes/r2000ic_165f/` 下，URDF 中统一采用 `meshes/r2000ic_165f/...` 相对路径引用，保证 `vamp_r2000ic/` 模块完全独立自包含、无需依赖外部环境路径。

### 步骤二：基于 Visual CAD 网格的高精度模型球体化逼近 (FOAM MAT Spherization)
为了使包络球严格贴合机械臂真实物理几何外形，本工程引入了 Rice 大学 Kavraki 实验室的 **`CoMMALab/foam`**（基于中轴变换 Medial Axis Transform 的最优多球体逼近算法）。

相比于低模简化碰撞网格（Collision STL），**直接从高分辨率 CAD 可视网格（Visual DAE，单连杆多达 7,000 ~ 26,000 顶点）进行球体化**能够消除多边形简化带来的外形形变，精准贴合减速机、关节凸台与法兰末端细节：

```bash
# 转换 Visual DAE 为标准流形网格并调用 FOAM MAT 算法进行多球体逼近
C:\Users\15760\miniconda3\envs\foam\python.exe scripts/run_foam_on_visual.py
```

该算法解算出 Fanuc R-2000iC/165F 全身 7 个连杆的高精度几何包络球（共 32 个球体，保存在 `models/r2000ic_165f_visual_foam_spheres.json` 及 `models/r2000ic_165f_spherized.urdf`）：
- **base_link (底座台架，4 球)**：大球半径 0.353 ~ 0.364m，小球半径 0.104m 精准覆盖地脚与安装耳。
- **J1_link (腰部回转立柱，4 球)**：球心分布于 `[-0.21, 0.26]`，半径 0.324 ~ 0.352m，紧密贴合回转电机与圆台。
- **J2_link (下大臂，6 球)**：沿大臂垂直伸展轴（Z 从 0.02m 到 1.13m）布置，半径 0.168 ~ 0.265m。
- **J3_link (肘部配重 + 前臂筒段，9 球)**：
  - 肘部与后置电机配重：4 个球（半径 0.135 ~ 0.300m），精准包络曲面轮廓；
  - 前臂钢管圆筒（X 从 0.35m 到 1.05m）：5 个精细紧致球（半径仅 0.110 ~ 0.132m，原先粗放求解为 0.245m，半径直接降低了一半，完美贴合圆筒截面并消除虚假膨胀）。
- **J4_link (前臂旋转轴套，4 球)**：沿 X 从 1.09m 到 1.33m 密集紧致排列，半径 0.098 ~ 0.112m，紧贴旋转体外壳。
- **J5_link (手腕俯仰机构，3 球)**：半径 0.093 ~ 0.142m，紧密贴合手腕双叉结构。
- **J6_link (法兰工具盘，2 球)**：半径仅 0.083m，紧贴法兰圆盘末端，为末端夹爪预留真实作业裕度。

同时在 `r2000ic_165f_spherized.urdf` 中保留了完整的 `<visual>` CAD 引用，既能在 3D 界面（RViz / WebGUI / Tesseract Viewer）中展示精致的真实工业机器人外观，又能在规划底层以极其贴切的高拟合球体进行超高速 SIMD 判定。

### 步骤三：自碰撞豁免矩阵 (ACM) 配置
在 `models/r2000ic_165f.srdf` 中定义了 Allowed Collision Matrix：
- 严格豁免物理相邻的连杆对（如 J1 与 J2、J4 与 J5 等）。
- 豁免空间上因机械限位永不可能相撞的连杆对（如 base 与 J3）。
- 运行时仅需检测非相邻且可能触碰的球体对，极大减轻内部检测开销。

### 步骤四：SIMD 向量化正运动学与碰撞内核
核心源码位于 `include/vamp_r2000ic/`：
- **`simd_defs.hpp`**：封装 AVX2 / SSE 向量指令。`check_sphere_aabb_collision` 函数通过向量寄存器计算球心到 AABB 的最近距离平方。
- **`r2000ic_simd_fk.hpp`**：展开计算 6 个旋转关节。给定 6 维关节角，直接求出全部连杆坐标系，并将局部球心批量投影为全局世界坐标。
- **`r2000ic_collision.hpp`**：维护静态/动态障碍物列表（Box、Sphere），提供单状态及批量状态（Batch）的高性能碰撞校验。

### 步骤五：OMPL 适配桥接与高层规划器
- **`vamp_ompl_adapter.hpp`**：
  - `createR2000icStateSpace()`：构建 6 自由度实数向量状态空间并硬编码 Fanuc R-2000iC/165F 的物理限位。
  - `VampStateValidityChecker`：继承 OMPL 的 `StateValidityChecker`，调用 SIMD 碰撞器。
  - `VampMotionValidator`：基于关节最大跨度插值密集检测直线边。
- **`vamp_r2000ic_planner.hpp / .cpp`**：
  - 封装干净简洁的高层 API：`init()`、`addObstacleBox()`、`removeObstacle()`、`planFreespace()`。

---

## 4. 如何在 RobotPlanner 中无缝接入（DLL 运行时动态加载）

为了保证主库 `robot_planner` 的轻量解耦与模块化，本工程采用 **纯 C-ABI 导出 DLL + 运行时动态加载** 的架构：
- 编译期：`robot_planner` **完全无需链接 `vamp_r2000ic.lib`**，零编译期硬依赖。
- 运行期：`robot_planner` 初始化时自动检测当前机型，若为 `r2000ic_165f`，则通过 `VampDynamicLoader` 尝试使用 Windows 原生 API（`LoadLibraryA` / `GetProcAddress`）动态加载 `vamp_r2000ic.dll`。
- 优雅降级：若动态库缺失或平台不支持，自动平滑退回 Tesseract 默认规划流水线，系统依然安全稳定。

```text
┌────────────────────────────────────────────────────────┐
│                   test_robot_planner                   │
└───────────────────────────┬────────────────────────────┘
                            │ (静态链接)
┌───────────────────────────▼────────────────────────────┐
│                      robot_planner                     │
│                                                        │
│  • Tesseract 核心 (Environment / Task Composer)        │
│  • VampDynamicLoader (动态加载器，无编译期硬链接)       │
└───────────────────────────┬────────────────────────────┘
                            │
               LoadLibraryA │ GetProcAddress
                            ▼ (运行时按需加载 C-ABI 接口)
┌────────────────────────────────────────────────────────┐
│                    vamp_r2000ic.dll                    │
│                                                        │
│  • 纯 C-ABI 导出: vamp_r2000ic_c_api.h                │
│  • 28-Sphere AVX2 SIMD 运动学与极速求交内核            │
│  • OMPL RRTConnect / RRT* / PRM 求解器                │
└────────────────────────────────────────────────────────┘
```

### 4.1 C-ABI 导出层与句柄管理
在 `include/vamp_r2000ic/vamp_r2000ic_c_api.h` 中提供纯 C 导出接口，彻底规避 MSVC C++ STL 跨 DLL 的 ABI 兼容性隐患：
```c
VAMP_C_API VampHandle vamp_r2000ic_create();
VAMP_C_API void vamp_r2000ic_destroy(VampHandle handle);
VAMP_C_API int vamp_r2000ic_init(VampHandle handle);
VAMP_C_API int vamp_r2000ic_add_box(VampHandle handle, const char* name, double x, double y, double z, double dx, double dy, double dz);
VAMP_C_API int vamp_r2000ic_remove_obstacle(VampHandle handle, const char* name);
VAMP_C_API int vamp_r2000ic_check_collision(VampHandle handle, const double* joints_rad);
VAMP_C_API int vamp_r2000ic_plan_freespace(VampHandle handle, const double* start_joints, const double* target_joints,
                                          double* out_waypoints, int max_waypoints, int* out_actual_waypoints,
                                          double timeout_sec, double range, double safety_margin, const char* planner_type);
```

### 4.2 轻量级运行时动态加载器 `VampDynamicLoader`
在 `include/robot_planner/vamp_dynamic_loader.h` 中实现：
```cpp
class VampDynamicLoader {
public:
    bool load(const std::string& custom_dll_path = "");
    void unload();
    bool isLoaded() const;
    // 转发 C 接口调用...
};
```
加载器会在当前可执行文件目录、`./`、`workspace/Release/` 等路径下自动搜寻 `vamp_r2000ic.dll` 并解析函数地址。

### 4.3 在 `RobotPlanner` 中按需激活与极速通道
在 `src/robot_planner.cpp` 中：
```cpp
// 1. 初始化时自动嗅探与动态加载
if (manipulator_name == "r2000ic_165f" || urdf_path.find("r2000ic") != std::string::npos) {
    pimpl_->vamp_loader_ = std::make_unique<VampDynamicLoader>();
    if (pimpl_->vamp_loader_->load()) {
        pimpl_->use_vamp_ = true;
        std::cout << "[RobotPlanner] Successfully loaded vamp_r2000ic.dll -> AVX2 SIMD hardware acceleration active!\n";
    }
}

// 2. 障碍物同步添加/移除
if (pimpl_->use_vamp_ && pimpl_->vamp_loader_) {
    pimpl_->vamp_loader_->addBox(name, x, y, z, dim_x, dim_y, dim_z);
}

// 3. 碰撞检测极速通道
if (pimpl_->use_vamp_ && pimpl_->vamp_loader_) {
    return pimpl_->vamp_loader_->checkCollision(joint_angles); // 仅 0.45 微秒
}

// 4. 自由避障规划毫秒级求解
if (pimpl_->use_vamp_ && pimpl_->vamp_loader_) {
    return pimpl_->vamp_loader_->planFreespace(start_joints, target_joints, trajectory_out, ...); // 仅需 1~5 ms
}
```

---

## 5. 编译与基准性能测试

### 5.1 独立构建
在当前目录或顶层工程中构建：
```powershell
cmake -B build -S .
cmake --build build --config Release --target test_vamp_benchmark
```

### 5.2 运行性能测试程序
```powershell
.\workspace\Release\test_vamp_benchmark.exe
```

测试实测输出（AMD/Intel AVX2 架构，单核实测）：
```text
=========================================================
   Fanuc R-2000iC/165F VAMP SIMD Accelerated Planning    
=========================================================
Planner successfully initialized with 6-DOF R-2000iC StateSpace.

[1] Running SIMD Forward Kinematics & Collision Benchmark...
  Iterations: 100000
  Total Time: 45.88 ms
  Speed:      0.459 microseconds per check
  Throughput: 2179442 checks/sec  (218 万次/秒)

[2] Planning Free Space Motion (No Obstacle)...
Info:    RRTConnect: Created 70 states (2 start + 68 goal)
Info:    Solution found in 0.000729 seconds
  -> Plan succeeded in: 1 ms
  -> Trajectory waypoints: 69

[3] Planning Obstacle Avoidance Motion (with Obstacle Box blocking direct path)...
Info:    RRTConnect: Created 225 states (103 start + 122 goal)
Info:    Solution found in 0.010064 seconds
  -> Obstacle Avoidance succeeded in: 10 ms
  -> Trajectory waypoints: 101

[4] Planning with RRT* (Optimal Planner)...
Info:    RRTstar: Created 5414 new states. Checked 13290398 rewire options. Final solution cost 1.475
Info:    Solution found in 2.202934 seconds
  -> RRT* Succeeded in: 2204.050 ms
  -> Trajectory waypoints: 31
```

---

## 6. 性能与优势对比

| 指标 | 传统模式 (Bullet / FCL + Mesh) | VAMP SIMD 加速模式 (本模块) | 提升幅度 |
| :--- | :--- | :--- | :--- |
| **单次全状态碰撞检测** | 20 ~ 100 微秒 ($\mu s$) | **0.45 微秒 ($\mu s$)** | **约 50 ~ 100 倍** |
| **单核检测吞吐量** | 10,000 ~ 50,000 次/秒 | **2,180,000 次/秒** | **约 100 倍** |
| **RRTConnect 自由空间规划** | 50 ~ 150 毫秒 ($ms$) | **0.7 ~ 1 毫秒 ($ms$)** | **约 100 倍** |
| **RRTConnect 复杂避障规划** | 100 ~ 500 毫秒 ($ms$) | **10 毫秒 ($ms$)** | **约 30 ~ 50 倍** |
| **硬件需求** | 普通 CPU | 支持 AVX2 的标准现代 CPU | 无需昂贵 GPU |
| **几何贴合精度** | 低模 STL 粗略逼近 | **Visual CAD 网格中轴变换 (FOAM MAT)** | 极佳贴合度，无虚假膨胀 |
| **适用场景** | 离线轨迹生成、精度校核 | **毫秒级在线实时重规划、人机避障** | 质的突破 |
