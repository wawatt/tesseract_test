#include "robot_planner/robot_planner.h"
#include <iostream>
#include <vector>
#include <filesystem>
#include <cstdlib>

int main(int argc, char** argv) {
    // 设置 TESSERACT_RESOURCE_PATH 环境变量，以便正确解析 package://tesseract_support/...
    _putenv_s("TESSERACT_RESOURCE_PATH", "D:/build/vcpkg/vcpkg_installed/x64-windows/share");

    robot_planner::RobotPlanner planner;
    
    // 我们假设当前目录在构建目录下，资源路径使用绝对路径
    std::string urdf_path = "D:/build/vcpkg/vcpkg_installed/x64-windows/share/tesseract/support/urdf/lbr_iiwa_14_r820.urdf";
    std::string srdf_path = "D:/build/vcpkg/vcpkg_installed/x64-windows/share/tesseract/support/urdf/lbr_iiwa_14_r820.srdf";
    
    std::cout << "Initializing planner..." << std::endl;
    // 使用KUKA iiwa模型，基座为 base_link，末端为 tool0
    if (!planner.init(urdf_path, srdf_path, "manipulator", "base_link", "tool0")) {
        std::cerr << "Initialization failed!" << std::endl;
        return -1;
    }
    std::cout << "Initialization successful!\n" << std::endl;
    
    // 1. 正向运动学 (FK) 测试
    std::cout << "--- 1. FK Test ---" << std::endl;
    std::vector<double> joint_angles = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> pose;
    if (planner.computeFK(joint_angles, pose)) {
        std::cout << "FK successful: Pos = [" << pose[0] << ", " << pose[1] << ", " << pose[2] 
                  << "], Quat = [" << pose[3] << ", " << pose[4] << ", " << pose[5] << ", " << pose[6] << "]" << std::endl;
    } else {
        std::cerr << "FK failed!" << std::endl;
    }
    
    // 2. 逆向运动学 (IK) 测试
    std::cout << "\n--- 2. IK Test ---" << std::endl;
    // 我们用上一步 FK 的结果作为目标位姿
    std::vector<double> seed_joints = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1}; // 给一个微小偏移作为种子
    std::vector<double> ik_joints;
    if (planner.computeIK(pose, seed_joints, ik_joints)) {
        std::cout << "IK successful. Joints = [";
        for (double j : ik_joints) std::cout << j << " ";
        std::cout << "]" << std::endl;
    } else {
        std::cerr << "IK failed!" << std::endl;
    }
    
    // 3. 场景操作与碰撞检测测试
    std::cout << "\n--- 3. Collision & Scene Test ---" << std::endl;
    bool col1 = planner.checkCollision(joint_angles);
    std::cout << "Collision at zero joints (Empty Scene)? " << (col1 ? "Yes" : "No") << std::endl;
    
    // 在机器人正上方添加一个障碍物盒子
    std::cout << "Adding an obstacle box directly above the robot..." << std::endl;
    planner.addBox("obstacle_box", 0.5, 0.5, 0.1, 0.0, 0.0, 0.8);
    
    bool col2 = planner.checkCollision(joint_angles);
    std::cout << "Collision at zero joints (With Box)? " << (col2 ? "Yes" : "No") << std::endl;
    
    // 移除障碍物
    std::cout << "Removing the obstacle box..." << std::endl;
    planner.removeObstacle("obstacle_box");

    // 测试添加 Mesh
    std::cout << "\nAdding a mesh obstacle (tetrahedron)..." << std::endl;
    std::vector<double> mesh_vertices = {
        0.0, 0.0, 0.0,
        0.5, 0.0, 0.0,
        0.0, 0.5, 0.0,
        0.0, 0.0, 0.5
    };
    std::vector<int> mesh_faces = {
        0, 1, 2,
        0, 1, 3,
        0, 2, 3,
        1, 2, 3
    };
    planner.addMesh("obstacle_mesh", mesh_vertices, mesh_faces, {0,0,0, 0,0,0,1});
    bool col_mesh = planner.checkCollision(joint_angles);
    std::cout << "Collision at zero joints (With Mesh)? " << (col_mesh ? "Yes" : "No") << std::endl;
    planner.removeObstacle("obstacle_mesh");

    // 测试添加 PointCloud
    std::cout << "\nAdding a PointCloud obstacle..." << std::endl;
    std::vector<double> pc_points;
    for (int i=0; i<10; ++i) {
        for (int j=0; j<10; ++j) {
            pc_points.push_back(0.0 + i*0.05);
            pc_points.push_back(0.0 + j*0.05);
            pc_points.push_back(0.5);
        }
    }
    planner.addPointCloud("obstacle_pc", pc_points, 0.05, {0,0,0, 0,0,0,1});
    bool col_pc = planner.checkCollision(joint_angles);
    std::cout << "Collision at zero joints (With PointCloud)? " << (col_pc ? "Yes" : "No") << std::endl;
    planner.removeObstacle("obstacle_pc");

    
    // 4. Freespace 运动规划测试 (对应 MoveInstructionType::FREESPACE)
    std::cout << "\n--- 4. Freespace Motion Planning Test ---" << std::endl;
    std::vector<double> target_joints = {0.5, 0.5, 0.0, -0.5, 0.0, 0.5, 0.0};
    std::vector<std::vector<double>> freespace_trajectory;
    if (planner.planFreespace(joint_angles, target_joints, freespace_trajectory)) {
        std::cout << "Freespace Planning successful! Trajectory points: " << freespace_trajectory.size() << std::endl;
        for (size_t i = 0; i < std::min<size_t>(3, freespace_trajectory.size()); ++i) {
            std::cout << "  Point " << i << ": ";
            for (double j : freespace_trajectory[i]) std::cout << j << " ";
            std::cout << std::endl;
        }
    } else {
        std::cerr << "Freespace Planning failed!" << std::endl;
    }
    
    // 5. Linear 笛卡尔直线规划测试 (对应 MoveInstructionType::LINEAR)
    std::cout << "\n--- 5. Linear Motion Planning Test ---" << std::endl;
    std::vector<double> target_pose_lin = pose;
    target_pose_lin[0] += 0.2; 
    
    std::vector<std::vector<double>> linear_trajectory;
    if (planner.planLinear(joint_angles, target_pose_lin, linear_trajectory)) {
        std::cout << "Linear Planning successful! Trajectory points: " << linear_trajectory.size() << std::endl;
        for (size_t i = 0; i < std::min<size_t>(3, linear_trajectory.size()); ++i) {
            std::cout << "  Point " << i << ": ";
            for (double j : linear_trajectory[i]) std::cout << j << " ";
            std::cout << std::endl;
        }
    } else {
        std::cerr << "Linear Planning failed! (Note: TrajOpt Linear requires fine-tuned profiles)" << std::endl;
    }
    
    // 6. Freespace 全局避障规划测试
    std::cout << "\n--- 6. Freespace Obstacle Avoidance Planning Test ---" << std::endl;
    std::cout << "Adding an obstacle box to block the path..." << std::endl;
    planner.addBox("obstacle_box_for_avoidance", 0.4, 0.4, 0.4, 0.2, 0.2, 0.7);
    
    std::vector<std::vector<double>> avoid_trajectory;
    // 传入自定义参数：15秒最大超时，0.02生长步长，0.04米安全边距，30.0碰撞系数，以及 RRTstar 算法
    if (planner.planFreespace(joint_angles, target_joints, avoid_trajectory, 15.0, 0.02, 0.04, 30.0, "RRTstar")) {
        std::cout << "Freespace Obstacle Avoidance Planning successful! Trajectory points: " << avoid_trajectory.size() << std::endl;
        for (size_t i = 0; i < std::min<size_t>(3, avoid_trajectory.size()); ++i) {
            std::cout << "  Point " << i << ": ";
            for (double j : avoid_trajectory[i]) std::cout << j << " ";
            std::cout << std::endl;
        }
    } else {
        std::cerr << "Freespace Obstacle Avoidance Planning failed!" << std::endl;
    }
    planner.removeObstacle("obstacle_box_for_avoidance");

    // 7. Circular 笛卡尔圆弧规划测试 (对应 MoveInstructionType::CIRCULAR)
    std::cout << "\n--- 7. Circular Motion Planning Test ---" << std::endl;
    std::vector<double> aux_pose = pose;
    aux_pose[0] += 0.1;
    aux_pose[1] += 0.05;
    std::vector<double> target_pose_circ = pose;
    target_pose_circ[0] += 0.2;
    
    std::vector<std::vector<double>> circular_trajectory;
    if (planner.planCircular(joint_angles, aux_pose, target_pose_circ, circular_trajectory)) {
        std::cout << "Circular Planning successful! Trajectory points: " << circular_trajectory.size() << std::endl;
        for (size_t i = 0; i < std::min<size_t>(3, circular_trajectory.size()); ++i) {
            std::cout << "  Point " << i << ": ";
            for (double j : circular_trajectory[i]) std::cout << j << " ";
            std::cout << std::endl;
        }
    } else {
        std::cerr << "Circular Planning failed! (Note: TrajOpt Circular requires fine-tuned profiles)" << std::endl;
    }

    // 8. 更多 Freespace 算法测试
    std::cout << "\n--- 8. More Freespace Algorithms Test ---" << std::endl;
    std::vector<std::vector<double>> traj_prm, traj_rrtconn;
    if (planner.planFreespace(joint_angles, target_joints, traj_prm, 10.0, 0.05, 0.025, 20.0, "PRM")) {
        std::cout << "PRM Planning successful!" << std::endl;
    } else {
        std::cerr << "PRM Planning failed!" << std::endl;
    }
    if (planner.planFreespace(joint_angles, target_joints, traj_rrtconn, 10.0, 0.01, 0.025, 20.0, "RRTConnect")) {
        std::cout << "RRTConnect Planning successful!" << std::endl;
    } else {
        std::cerr << "RRTConnect Planning failed!" << std::endl;
    }
    
    // 测试无效的 planner type
    std::vector<std::vector<double>> traj_invalid;
    if (planner.planFreespace(joint_angles, target_joints, traj_invalid, 10.0, 0.01, 0.025, 20.0, "UNKNOWN_ALGO")) {
        std::cout << "UNKNOWN_ALGO Planning successful (fallback expected)!" << std::endl;
    } else {
        std::cout << "UNKNOWN_ALGO Planning failed (as expected or fallback failed)!" << std::endl;
    }

    // 9. 无效输入测试
    std::cout << "\n--- 9. Invalid Input Test ---" << std::endl;
    std::vector<double> empty_joints;
    std::vector<double> bad_pose = {0.0, 0.0};
    if (!planner.computeFK(empty_joints, pose)) {
        std::cout << "FK handled empty joints correctly (returned false)." << std::endl;
    }
    if (!planner.computeIK(bad_pose, joint_angles, ik_joints)) {
        std::cout << "IK handled bad pose correctly (returned false)." << std::endl;
    }

    // 10. 路径上有障碍物的 Linear 规划测试
    std::cout << "\n--- 10. Linear Planning with Obstacle in Path ---" << std::endl;
    // 添加障碍物，正挡在目标和当前位置之间
    planner.addBox("block_box_lin", pose[0] + 0.1, pose[1], pose[2], 0.2, 0.2, 0.2);
    std::vector<std::vector<double>> lin_fail_traj;
    if (planner.planLinear(joint_angles, target_pose_lin, lin_fail_traj)) {
        std::cerr << "Linear Planning succeeded but should have failed due to obstacle!" << std::endl;
    } else {
        std::cout << "Linear Planning correctly failed when obstacle blocks the path." << std::endl;
    }
    planner.removeObstacle("block_box_lin");

    // 11. 路径上有障碍物的 Circular 规划测试
    std::cout << "\n--- 11. Circular Planning with Obstacle in Path ---" << std::endl;
    // 添加障碍物，正挡在圆弧中间点
    planner.addBox("block_box_circ", aux_pose[0], aux_pose[1], aux_pose[2], 0.2, 0.2, 0.2);
    std::vector<std::vector<double>> circ_fail_traj;
    if (planner.planCircular(joint_angles, aux_pose, target_pose_circ, circ_fail_traj)) {
        std::cerr << "Circular Planning succeeded but should have failed due to obstacle!" << std::endl;
    } else {
        std::cout << "Circular Planning correctly failed when obstacle blocks the path." << std::endl;
    }
    planner.removeObstacle("block_box_circ");

    std::cout << "\nDemo finished successfully." << std::endl;
    return 0;
}
