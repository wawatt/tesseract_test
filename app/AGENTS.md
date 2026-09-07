# 工业机器人三维运动仿真与示教工作站 (Robot Simulation Workstation)

基于 **Qt5** + **OpenSceneGraph (osgQOpenGLWidget)** + **robot_planner** 封装的高性能 3D 机器人运动仿真与交互式示教桌面软件。

## 依赖环境

- **编译工具链**: `CMAKE_TOOLCHAIN_FILE=D:/build/vcpkg/vcpkg_installed/scripts/buildsystems/vcpkg.cmake`
- **Qt5**: Qt5 Widgets, Core, Gui, OpenGL (`unofficial-osg` / `osg-qt`)
- **OpenSceneGraph**: OSG 3.6.5 (`osgQOpenGLWidget`, `osgDB`, `osgViewer`, `osgGA`, `osgUtil`, `osgText`)
- **Mesh 转换引擎**: Assimp
- **运动学与规划引擎**: `robot_planner` (结合 VAMP AVX2 SIMD 硬件加速)

## 核心功能与架构

1. **3D 机器人模型与视口 (`OsgViewerWidget`, `RobotVisualNode`)**
   - 基于 `osgQOpenGLWidget` 构建嵌入式高性能 OpenGL 视口。
   - 动态加载解析机械臂 3D 网格模型 (`.dae`, `.obj`, `.stl`)，各连杆挂载到 `osg::MatrixTransform`。
   - 随正向运动学 (FK) 毫秒级刷新连杆世界位姿与末端 TCP 坐标系。
   - 工业科技深灰色地表坐标网格与世界坐标三轴指示。

2. **多轴与笛卡尔交互式示教 (`JogPanelDock`)**
   - **关节单轴示教 (Joint Jog)**: 6 轴角度滑块微调，实时限位保护与度数显示，支持一键归位 (Home / Ready / Zero)。
   - **笛卡尔末端点动 (Cartesian Jog)**: 实时计算并显示 TCP 笛卡尔坐标 `[X, Y, Z, Rx, Ry, Rz]`，支持 1mm~100mm、1°~45° 步长单轴步进点动与逆解追踪。

3. **3D 场景与障碍物管理 (`SceneTreeDock`, `AddObstacleDialog`)**
   - 场景树统一展示机械臂连杆树与场景障碍物。
   - 交互式对话框添加立方体 (Box)、球体 (Sphere)、圆柱体 (Cylinder) 及外部网格 (Mesh) 障碍物。
   - 障碍物右键菜单支持：工件抓取挂载到末端 (`Attach to Tool0`)、脱离放回世界坐标系 (`Detach`)、删除障碍物。

4. **运动规划与动作序列 (`PlanningDock`)**
   - **点位库 (Waypoints)**: 支持保存、重命名、删除及一键跳转到当前示教位姿点。
   - **多模式运动规划**:
     - 自由空间避障规划 (PTP / Freespace)
     - 笛卡尔空间直线插补 (LIN / Linear)
     - 笛卡尔三点圆弧插补 (CIRC / Circular)
   - 动力学与安全参数配置：速度缩放比、加速度缩放比、安全净距裕度、规划超时时间。

5. **轨迹时间轴与 60FPS 动画回放 (`TimelineDock`)**
   - 轨迹播放进度滑块、播放/暂停、停止、循环播放与 0.25x~5.0x 倍速调节。
   - 动力学平滑时间参数化轨迹线性与三阶插值回放。

6. **实时碰撞干涉变红高亮与诊断 (`LogDock`)**
   - 示教或回放时，发生碰撞的连杆与障碍物在 3D 视口中立即变红高亮警示。
   - 绘制碰撞接触点（黄色）与法线箭头（红色）。
   - 诊断面板实时输出干涉对、侵入深度与安全裕度。

7. **现代深色工业主题 (`dark_theme.qss`)**
   - 定制深灰 Slate 配色、科技蓝控件高亮、扁平化微交互设计。

## 编译与运行

### 编译应用
```powershell
cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=D:/build/vcpkg/vcpkg_installed/scripts/buildsystems/vcpkg.cmake
cmake --build build --config Release --target robot_sim_app
```

### 运行可执行文件
可执行文件自动输出并部署至 `workspace/Release/robot_sim_app.exe`：
```powershell
.\workspace\Release\robot_sim_app.exe
```

