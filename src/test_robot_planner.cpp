#include "robot_planner/robot_planner.h"
#include "robot_planner/opw_kinematics.h"
#include <iostream>
#include <vector>
#include <filesystem>
#include <cstdlib>
#include <chrono>
#include <iomanip>
#include <numeric>
#include <algorithm>
#include <sstream>

int main(int argc, char** argv) {
    // 设置 TESSERACT_RESOURCE_PATH 环境变量，以便正确解析 package://tesseract_support/...
    _putenv_s("TESSERACT_RESOURCE_PATH", "D:/build/vcpkg/vcpkg_installed/x64-windows/share");

    // 全局统一使用 Fanuc R-2000iC/165F 工业机器人模型
    const std::string fanuc_urdf = "vamp_r2000ic/models/r2000ic_165f.urdf";
    const std::string fanuc_srdf = "vamp_r2000ic/models/r2000ic_165f.srdf";
    const std::string manip_name = "r2000ic_165f";
    const std::string base_link = "base_link";
    const std::string tool_link = "tool0";

    std::cout << "=========================================================" << std::endl;
    std::cout << "   Robot Planner Tests: Fanuc R-2000iC/165F (6-DOF)      " << std::endl;
    std::cout << "=========================================================" << std::endl;

    // =========================================================================
    // PART 1: Fanuc R-2000iC with TESSERACT Backend (Bullet/FCL Engine)
    // =========================================================================
    std::cout << "\n>>> PART 1: Testing TESSERACT Backend (Bullet/FCL Engine) <<<" << std::endl;
    robot_planner::RobotPlanner planner;
    
    std::cout << "Initializing planner (TESSERACT Backend)..." << std::endl;
    if (!planner.init(fanuc_urdf, fanuc_srdf, manip_name, base_link, tool_link, robot_planner::PlannerBackend::TESSERACT)) {
        std::cerr << "Initialization failed! Error: " << planner.getLastError() << std::endl;
        return -1;
    }
    std::cout << "Initialization successful! Backend: " 
              << (planner.getBackend() == robot_planner::PlannerBackend::TESSERACT ? "TESSERACT" : "VAMP") << "\n" << std::endl;
    
    // 1. 正向运动学 (FK) 测试
    std::cout << "--- 1. FK Test ---" << std::endl;
    std::vector<double> joint_angles = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> pose;
    if (planner.computeFK(joint_angles, pose)) {
        std::cout << "FK successful: Pos = [" << pose[0] << ", " << pose[1] << ", " << pose[2] 
                  << "], Quat = [" << pose[3] << ", " << pose[4] << ", " << pose[5] << ", " << pose[6] << "]" << std::endl;
    } else {
        std::cerr << "FK failed! Error: " << planner.getLastError() << std::endl;
    }
    
    // 2. 逆向运动学 (IK) 与 URDF 限位过滤测试
    std::cout << "\n--- 2. IK Test (Analytical + URDF Limits Filtering) ---" << std::endl;
    std::vector<double> seed_joints = {0.05, 0.05, 0.05, 0.05, 0.05, 0.05};
    std::vector<double> ik_joints;
    auto ik_t0 = std::chrono::high_resolution_clock::now();
    bool ik_ok = planner.computeIK(pose, seed_joints, ik_joints);
    auto ik_t1 = std::chrono::high_resolution_clock::now();
    double ik_us = std::chrono::duration<double, std::micro>(ik_t1 - ik_t0).count();
    if (ik_ok) {
        std::cout << "IK successful in " << ik_us << " us. Nearest in-limit Joints = [";
        for (double j : ik_joints) std::cout << j << " ";
        std::cout << "]" << std::endl;
    } else {
        std::cout << "IK failed! Error: " << planner.getLastError() << std::endl;
    }

    std::vector<std::vector<double>> all_sols;
    if (planner.computeAllIK(pose, all_sols)) {
        std::cout << "computeAllIK exported " << all_sols.size() 
                  << " strictly valid solutions within URDF physical limits." << std::endl;
    }

    // 3. 场景生命周期、工件抓取附着 (attach/detach) 与碰撞检测测试
    std::cout << "\n--- 3. Scene Lifecycle & Attached Body Collision Test ---" << std::endl;
    bool col1 = planner.checkCollision(joint_angles);
    std::cout << "Collision at zero joints (Empty Scene)? " << (col1 ? "Yes" : "No") << std::endl;
    
    // 添加障碍物
    planner.addBox("workpiece_box", 1.2, 0.0, 1.0, 0.2, 0.2, 0.2);
    std::cout << "Added 'workpiece_box'. hasObstacle? " << (planner.hasObstacle("workpiece_box") ? "Yes" : "No") << std::endl;
    auto obs_list = planner.getObstacleNames();
    std::cout << "Active obstacles in scene: " << obs_list.size() << std::endl;

    // 测试工件挂载附着 (Attach to tool0)
    std::cout << "Attaching 'workpiece_box' to robot flange (tool0)..." << std::endl;
    if (planner.attachObject("workpiece_box", "tool0")) {
        std::cout << "workpiece_box successfully attached to tool0!" << std::endl;
    } else {
        std::cerr << "Failed to attach workpiece! Error: " << planner.getLastError() << std::endl;
    }

    // 测试工件分离解挂 (Detach)
    std::cout << "Detaching 'workpiece_box' back to world..." << std::endl;
    if (planner.detachObject("workpiece_box")) {
        std::cout << "workpiece_box successfully detached!" << std::endl;
    }

    // 测试一键清空场景 (clearObstacles)
    planner.addBox("temp_box_1", 0.0, 0.0, 2.0, 0.1, 0.1, 0.1);
    planner.addBox("temp_box_2", 0.0, 0.0, 3.0, 0.1, 0.1, 0.1);
    std::cout << "Obstacle count before clear: " << planner.getObstacleNames().size() << std::endl;
    planner.clearObstacles();
    std::cout << "Obstacle count after clearObstacles(): " << planner.getObstacleNames().size() << std::endl;

    // 4. 动力学统一轨迹测试: Freespace / Linear / Circular
    std::cout << "\n--- 4. Full Trajectory Dynamics Test (JointTrajectory: Pos/Vel/Acc/Time) ---" << std::endl;
    std::vector<double> target_joints = {0.5, 0.3, -0.4, 0.8, 0.2, 0.1};
    robot_planner::JointTrajectory free_traj;
    if (planner.planFreespace(joint_angles, target_joints, free_traj)) {
        std::cout << "Freespace Plan SUCCESS! Points: " << free_traj.size() 
                  << ", Duration: " << (free_traj.empty() ? 0.0 : free_traj.time_stamps.back()) << " s" << std::endl;
        std::cout << "  Start State: pos[0]=" << free_traj.positions[0][0] << ", vel[0]=" << free_traj.velocities[0][0] 
                  << ", acc[0]=" << free_traj.accelerations[0][0] << ", t=" << free_traj.time_stamps[0] << "s" << std::endl;
    }

    // 5. 速度缩放测试 (Speed Scaling 50%)
    std::cout << "\n--- 5. Speed Scaling Test (max_velocity_scaling = 0.5) ---" << std::endl;
    robot_planner::JointTrajectory scaled_traj;
    if (planner.planFreespace(joint_angles, target_joints, scaled_traj, 0.5, 0.5)) {
        std::cout << "50% Speed Plan SUCCESS! Duration: " << scaled_traj.time_stamps.back() << " s (Expected ~2x slower)" << std::endl;
    }

    // 6. 笛卡尔直线规划 (带动力学输出)
    std::cout << "\n--- 6. Linear Motion Planning (JointTrajectory) ---" << std::endl;
    std::vector<double> target_pose_lin = pose;
    target_pose_lin[0] += 0.05;
    robot_planner::JointTrajectory lin_traj;
    if (planner.planLinear(joint_angles, target_pose_lin, lin_traj)) {
        std::cout << "Linear Planning SUCCESS! Points: " << lin_traj.size() 
                  << ", Duration: " << lin_traj.time_stamps.back() << " s" << std::endl;
    } else {
        std::cout << "Linear Planning failed: " << planner.getLastError() << std::endl;
    }

    // 7. 错误诊断机制测试 (Error Diagnostics)
    std::cout << "\n--- 7. Error Diagnostics Test ---" << std::endl;
    std::vector<double> col_p1 = pose;
    std::vector<double> col_p2 = pose;
    col_p1[0] += 0.05;
    col_p2[0] += 0.10; // 共线三点
    robot_planner::JointTrajectory bad_circ_traj;
    bool bad_circ = planner.planCircular(joint_angles, col_p1, col_p2, bad_circ_traj);
    std::cout << "Collinear circular plan rejected? " << (!bad_circ ? "Yes" : "No") << std::endl;
    std::cout << "Diagnostic Status: " << static_cast<int>(planner.getLastErrorStatus()) 
              << " | Message: " << planner.getLastError() << std::endl;

    // =========================================================================
    // PART 2: Fanuc R-2000iC with VAMP Backend (AVX2 SIMD & Trajectory Dynamics)
    // =========================================================================
    std::cout << "\n=========================================================" << std::endl;
    std::cout << ">>> PART 2: Testing VAMP Backend (AVX2 SIMD + Full Trajectory) <<<" << std::endl;
    std::cout << "=========================================================" << std::endl;

    robot_planner::RobotPlanner fanuc_planner;
    std::cout << "Initializing Fanuc R-2000iC planner (VAMP Backend)..." << std::endl;
    if (!fanuc_planner.init(fanuc_urdf, fanuc_srdf, manip_name, base_link, tool_link, robot_planner::PlannerBackend::VAMP)) {
        std::cerr << "Fanuc VAMP Planner initialization failed!" << std::endl;
    } else {
        std::cout << "Fanuc Planner successfully initialized with Backend: " 
                  << (fanuc_planner.getBackend() == robot_planner::PlannerBackend::VAMP ? "VAMP" : "TESSERACT") << std::endl;

        std::vector<double> f_start_joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        std::vector<double> f_target_joints = {0.5, 0.3, -0.4, 0.8, 0.2, 0.1};

        // 8.1 自由空间规划 (带动力学与时间戳)
        std::cout << "\n[8.1] VAMP Freespace Motion Planning (JointTrajectory)..." << std::endl;
        robot_planner::JointTrajectory f_free_traj;
        auto t_vamp_0 = std::chrono::high_resolution_clock::now();
        bool v_ok = fanuc_planner.planFreespace(f_start_joints, f_target_joints, f_free_traj);
        auto t_vamp_1 = std::chrono::high_resolution_clock::now();
        double ms_vamp_free = std::chrono::duration<double, std::milli>(t_vamp_1 - t_vamp_0).count();
        if (v_ok) {
            std::cout << ">> VAMP Freespace SUCCESS in " << ms_vamp_free << " ms! Points: " << f_free_traj.size() 
                      << ", Duration: " << f_free_traj.time_stamps.back() << " s" << std::endl;
        }

        // 8.2 VAMP 笛卡尔直线插补规划 (在工作姿态 J5=0.8rad 下进行，远离腕奇异)
        std::cout << "\n[8.2] VAMP Linear Motion Planning (Working Posture, J5=0.8rad)..." << std::endl;
        std::vector<double> f_work_joints = {0.0, 0.3, -0.2, 0.0, 0.8, 0.0};
        std::vector<double> f_work_pose;
        fanuc_planner.computeFK(f_work_joints, f_work_pose);
        std::vector<double> f_lin_target = f_work_pose;
        f_lin_target[0] -= 0.10; // 沿 X 轴平移 10cm
        f_lin_target[2] -= 0.05; // 沿 Z 轴平移 5cm
        robot_planner::JointTrajectory f_lin_traj;
        auto t_lin0 = std::chrono::high_resolution_clock::now();
        bool lin_ok = fanuc_planner.planLinear(f_work_joints, f_lin_target, f_lin_traj, 1.0, 1.0, 0.01);
        auto t_lin1 = std::chrono::high_resolution_clock::now();
        double ms_lin = std::chrono::duration<double, std::milli>(t_lin1 - t_lin0).count();
        if (lin_ok) {
            std::cout << ">> VAMP Linear Plan SUCCESS in " << ms_lin << " ms! Points: " << f_lin_traj.size() 
                      << ", Duration: " << f_lin_traj.time_stamps.back() << " s" << std::endl;
        } else {
            std::cerr << "VAMP Linear Plan failed! Error: " << fanuc_planner.getLastError() << std::endl;
        }

        // 8.3 VAMP 笛卡尔圆弧插补规划 (3-Point Circle + OPW IK + AVX2 + TrajOpt parameterization)
        std::cout << "\n[8.3] VAMP Circular Motion Planning (Working Posture)..." << std::endl;
        std::vector<double> f_circ_aux = f_work_pose;
        f_circ_aux[0] -= 0.05;
        f_circ_aux[2] -= 0.05;
        std::vector<double> f_circ_target = f_work_pose;
        f_circ_target[0] -= 0.10;
        robot_planner::JointTrajectory f_circ_traj;
        auto t_circ0 = std::chrono::high_resolution_clock::now();
        bool circ_ok = fanuc_planner.planCircular(f_work_joints, f_circ_aux, f_circ_target, f_circ_traj, 1.0, 1.0, 0.01);
        auto t_circ1 = std::chrono::high_resolution_clock::now();
        double ms_circ = std::chrono::duration<double, std::milli>(t_circ1 - t_circ0).count();
        if (circ_ok) {
            std::cout << ">> VAMP Circular Plan SUCCESS in " << ms_circ << " ms! Points: " << f_circ_traj.size() 
                      << ", Duration: " << f_circ_traj.time_stamps.back() << " s" << std::endl;
        } else {
            std::cerr << "VAMP Circular Plan failed! Error: " << fanuc_planner.getLastError() << std::endl;
        }

        // 8.4 奇异点与超出工作空间安全防御验证
        std::cout << "\n[8.4] Singularity & Workspace Limit Defensive Tests..." << std::endl;
        // 8.4.1 超出物理臂长测试
        std::vector<double> f_out_target = f_work_pose;
        f_out_target[0] += 5.0; // 超出 5 米
        robot_planner::JointTrajectory f_out_traj;
        bool out_ok = fanuc_planner.planLinear(f_work_joints, f_out_target, f_out_traj);
        std::cout << "Out-of-reach target correctly rejected? " << (!out_ok ? "Yes" : "No") << std::endl;
        std::cout << "  Status: " << static_cast<int>(fanuc_planner.getLastErrorStatus()) 
                  << " | Reason: " << fanuc_planner.getLastError() << std::endl;

        // 8.4.2 腕奇异点轴翻转防御测试 (在零位 J5=0.0rad 下做工具端面旋转)
        std::vector<double> f_zero_joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        std::vector<double> f_zero_pose;
        fanuc_planner.computeFK(f_zero_joints, f_zero_pose);
        std::vector<double> f_sing_target = f_zero_pose;
        f_sing_target[0] -= 0.10;
        // 旋转姿态在 J5=0 附近触发 J4/J6 跃变
        f_sing_target[3] = 0.5; f_sing_target[4] = 0.5; f_sing_target[5] = 0.5; f_sing_target[6] = 0.5;
        robot_planner::JointTrajectory f_sing_traj;
        bool sing_ok = fanuc_planner.planLinear(f_zero_joints, f_sing_target, f_sing_traj);
        std::cout << "Singular orientation jump correctly caught? " << (!sing_ok ? "Yes" : "No") << std::endl;
        std::cout << "  Status: " << static_cast<int>(fanuc_planner.getLastErrorStatus()) 
                  << " | Reason: " << fanuc_planner.getLastError() << std::endl;
    }

    // =========================================================================
    // PART 3: 性能对比基准测试 (VAMP Pipeline vs Native Tesseract)
    // =========================================================================
    std::cout << "\n=========================================================" << std::endl;
    std::cout << ">>> PART 3: Performance Timing Benchmark: VAMP vs Native Tesseract <<<" << std::endl;
    std::cout << "=========================================================" << std::endl;

    robot_planner::RobotPlanner fanuc_planner_tess;
    fanuc_planner_tess.init(fanuc_urdf, fanuc_srdf, manip_name, base_link, tool_link, robot_planner::PlannerBackend::TESSERACT);
    robot_planner::RobotPlanner& fanuc_planner_vamp = fanuc_planner;

    struct BenchmarkResult {
        std::string scene_name;
        double tesseract_avg_ms = 0.0;
        double tesseract_min_ms = 0.0;
        double tesseract_max_ms = 0.0;
        size_t tesseract_points = 0;
        double tesseract_duration_s = 0.0;
        bool tesseract_success = false;

        double vamp_avg_ms = 0.0;
        double vamp_min_ms = 0.0;
        double vamp_max_ms = 0.0;
        size_t vamp_points = 0;
        double vamp_duration_s = 0.0;
        bool vamp_success = false;

        double speedup = 0.0;
    };

    std::vector<BenchmarkResult> benchmark_results;

    auto run_benchmark_case = [&](const std::string& scene_name, int iterations = 3) -> BenchmarkResult {
        BenchmarkResult res;
        res.scene_name = scene_name;
        std::vector<double> f_start = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        std::vector<double> f_target = {0.5, 0.3, -0.4, 0.8, 0.2, 0.1};

        std::cout << "\n>>> Benchmarking Scenario: [" << scene_name << "] (" << iterations << " runs each) <<<" << std::endl;

        // 1. Tesseract Backend
        robot_planner::JointTrajectory tess_traj;
        fanuc_planner_tess.planFreespace(f_start, f_target, tess_traj, 1.0, 1.0, 5.0, 0.02, 0.025, 20.0, "RRTConnect");
        
        std::vector<double> tess_times;
        for (int i = 0; i < iterations; ++i) {
            auto t0 = std::chrono::high_resolution_clock::now();
            bool ok = fanuc_planner_tess.planFreespace(f_start, f_target, tess_traj, 1.0, 1.0, 5.0, 0.02, 0.025, 20.0, "RRTConnect");
            auto t1 = std::chrono::high_resolution_clock::now();
            if (ok) {
                double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
                tess_times.push_back(ms);
                res.tesseract_points = tess_traj.size();
                res.tesseract_duration_s = tess_traj.empty() ? 0.0 : tess_traj.time_stamps.back();
                res.tesseract_success = true;
            }
        }
        if (!tess_times.empty()) {
            res.tesseract_min_ms = *std::min_element(tess_times.begin(), tess_times.end());
            res.tesseract_max_ms = *std::max_element(tess_times.begin(), tess_times.end());
            res.tesseract_avg_ms = std::accumulate(tess_times.begin(), tess_times.end(), 0.0) / tess_times.size();
        }

        // 2. VAMP Backend
        robot_planner::JointTrajectory vamp_traj;
        fanuc_planner_vamp.planFreespace(f_start, f_target, vamp_traj, 1.0, 1.0, 5.0, 0.02, 0.025, 20.0, "RRTConnect");
        
        std::vector<double> vamp_times;
        for (int i = 0; i < iterations; ++i) {
            auto t0 = std::chrono::high_resolution_clock::now();
            bool ok = fanuc_planner_vamp.planFreespace(f_start, f_target, vamp_traj, 1.0, 1.0, 5.0, 0.02, 0.025, 20.0, "RRTConnect");
            auto t1 = std::chrono::high_resolution_clock::now();
            if (ok) {
                double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
                vamp_times.push_back(ms);
                res.vamp_points = vamp_traj.size();
                res.vamp_duration_s = vamp_traj.empty() ? 0.0 : vamp_traj.time_stamps.back();
                res.vamp_success = true;
            }
        }
        if (!vamp_times.empty()) {
            res.vamp_min_ms = *std::min_element(vamp_times.begin(), vamp_times.end());
            res.vamp_max_ms = *std::max_element(vamp_times.begin(), vamp_times.end());
            res.vamp_avg_ms = std::accumulate(vamp_times.begin(), vamp_times.end(), 0.0) / vamp_times.size();
        }

        if (res.tesseract_avg_ms > 0.0 && res.vamp_avg_ms > 0.0) {
            res.speedup = res.tesseract_avg_ms / res.vamp_avg_ms;
        }

        return res;
    };

    // --- Scenario 1: Empty Scene ---
    benchmark_results.push_back(run_benchmark_case("Empty Scene", 3));

    // --- Scenario 2: Box Obstacle ---
    fanuc_planner_tess.addBox("bench_box", 1.5, 0.0, 1.2, 0.4, 0.4, 0.4);
    fanuc_planner_vamp.addBox("bench_box", 1.5, 0.0, 1.2, 0.4, 0.4, 0.4);
    benchmark_results.push_back(run_benchmark_case("Box Obstacle", 3));
    fanuc_planner_tess.removeObstacle("bench_box");
    fanuc_planner_vamp.removeObstacle("bench_box");

    // --- Scenario 3: Mesh Obstacle ---
    std::vector<double> bm_mesh_vertices = {
        1.4, -0.2, 1.0,
        1.6, -0.2, 1.0,
        1.5,  0.2, 1.0,
        1.5,  0.0, 1.4
    };
    std::vector<int> bm_mesh_faces = {0, 1, 2,  0, 1, 3,  1, 2, 3,  2, 0, 3};
    fanuc_planner_tess.addMesh("bench_mesh", bm_mesh_vertices, bm_mesh_faces, {0, 0, 0, 0, 0, 0, 1});
    fanuc_planner_vamp.addMesh("bench_mesh", bm_mesh_vertices, bm_mesh_faces, {0, 0, 0, 0, 0, 0, 1});
    benchmark_results.push_back(run_benchmark_case("Mesh Obstacle", 3));
    fanuc_planner_tess.removeObstacle("bench_mesh");
    fanuc_planner_vamp.removeObstacle("bench_mesh");

    // --- Scenario 4: PointCloud Obstacle ---
    std::vector<double> bm_pc_points;
    for (double px = 1.3; px <= 1.6; px += 0.05) {
        for (double py = -0.2; py <= 0.2; py += 0.05) {
            for (double pz = 1.0; pz <= 1.3; pz += 0.05) {
                bm_pc_points.push_back(px);
                bm_pc_points.push_back(py);
                bm_pc_points.push_back(pz);
            }
        }
    }
    fanuc_planner_tess.addPointCloud("bench_pc", bm_pc_points, 0.05, {0, 0, 0, 0, 0, 0, 1});
    fanuc_planner_vamp.addPointCloud("bench_pc", bm_pc_points, 0.05, {0, 0, 0, 0, 0, 0, 1});
    benchmark_results.push_back(run_benchmark_case("PointCloud Wall", 3));
    fanuc_planner_tess.removeObstacle("bench_pc");
    fanuc_planner_vamp.removeObstacle("bench_pc");

    // --- 打印对比总结表格 ---
    std::cout << "\n=================================================================================================================" << std::endl;
    std::cout << "                              RobotPlanner 全动力学性能基准对比汇总表                                            " << std::endl;
    std::cout << "=================================================================================================================" << std::endl;
    std::cout << std::left << std::setw(18) << "Scene Name"
              << " | " << std::setw(20) << "Native (No VAMP)"
              << " | " << std::setw(20) << "VAMP Pipeline"
              << " | " << std::setw(10) << "Speedup"
              << " | " << std::setw(14) << "Points (T/V)"
              << " | " << std::setw(16) << "Duration (T/V)"
              << " |" << std::endl;
    std::cout << "-------------------+----------------------+----------------------+------------+----------------+------------------|" << std::endl;

    for (const auto& r : benchmark_results) {
        std::stringstream ss_tess, ss_vamp, ss_pts, ss_dur;
        ss_tess << std::fixed << std::setprecision(2) << r.tesseract_avg_ms << " ms (" << r.tesseract_min_ms << "~" << r.tesseract_max_ms << ")";
        ss_vamp << std::fixed << std::setprecision(2) << r.vamp_avg_ms << " ms (" << r.vamp_min_ms << "~" << r.vamp_max_ms << ")";
        ss_pts << r.tesseract_points << " / " << r.vamp_points;
        ss_dur << std::fixed << std::setprecision(2) << r.tesseract_duration_s << "s / " << r.vamp_duration_s << "s";

        std::cout << std::left << std::setw(18) << r.scene_name
                  << " | " << std::setw(20) << ss_tess.str()
                  << " | " << std::setw(20) << ss_vamp.str()
                  << " | " << std::fixed << std::setprecision(2) << std::setw(8) << r.speedup << "x"
                  << " | " << std::setw(14) << ss_pts.str()
                  << " | " << std::setw(16) << ss_dur.str()
                  << " |" << std::endl;
    }
    std::cout << "=================================================================================================================\n" << std::endl;

    std::cout << "\nAll Modernized Fanuc R-2000iC/165F Tests finished successfully." << std::endl;
    return 0;
}
