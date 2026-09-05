#pragma once

#include <vector>
#include <array>
#include <cmath>
#include <limits>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opw_kinematics/opw_kinematics.h>
#include <opw_kinematics/opw_utilities.h>

namespace robot_planner {

/**
 * @brief OPW (Orthogonal-Parallel-Wrist) 工业六轴机器人微秒级闭式解析运动学求解器
 * 针对 Fanuc R-2000iC/165F 以及同构型 6-DOF 工业机器人提供 < 1 微秒极速正逆向解算。
 */
class OpwKinematics {
public:
    static opw_kinematics::Parameters<double> getFanucR2000iCParameters() {
        opw_kinematics::Parameters<double> p;
        p.a1 = 0.312;
        p.a2 = -0.225;
        p.b  = 0.0;
        p.c1 = 0.670;
        p.c2 = 1.075;
        p.c3 = 1.280;
        p.c4 = 0.215;

        p.offsets[0] = 0.0;
        p.offsets[1] = 0.0;
        p.offsets[2] = -M_PI / 2.0;
        p.offsets[3] = 0.0;
        p.offsets[4] = 0.0;
        p.offsets[5] = 0.0;

        p.sign_corrections[0] = 1;
        p.sign_corrections[1] = 1;
        p.sign_corrections[2] = -1;
        p.sign_corrections[3] = -1;
        p.sign_corrections[4] = -1;
        p.sign_corrections[5] = -1;
        return p;
    }

    static opw_kinematics::Parameters<double> fromJointOrigins(const std::vector<double>& origins_xyz) {
        auto p = getFanucR2000iCParameters();
        if (origins_xyz.size() >= 18) {
            p.c1 = origins_xyz[0 * 3 + 2]; // J1 origin.z
            p.a1 = origins_xyz[1 * 3 + 0]; // J2 origin.x
            p.c2 = origins_xyz[2 * 3 + 2]; // J3 origin.z
            if (std::abs(origins_xyz[3 * 3 + 2]) > 1e-4) {
                p.a2 = -std::abs(origins_xyz[3 * 3 + 2]);
            }
            p.c3 = origins_xyz[4 * 3 + 0]; // J5 origin.x
        }
        return p;
    }

    /**
     * @brief 正向运动学 (FK)
     */
    static bool computeFK(const opw_kinematics::Parameters<double>& params,
                          const std::vector<double>& joint_angles,
                          std::vector<double>& pose_out) {
        if (joint_angles.size() < 6) return false;
        std::array<double, 6> qs;
        for (int i = 0; i < 6; ++i) qs[i] = joint_angles[i];

        auto T = opw_kinematics::forward(params, qs);
        // OPW flange frame has Z pointing along tool. Fanuc URDF tool0 frame has X pointing along tool.
        // Transform: R_urdf = R_opw * Ry(-pi/2)
        Eigen::Matrix3d R_urdf = T.linear() * Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitY());
        Eigen::Quaterniond q(R_urdf);

        pose_out.resize(7);
        pose_out[0] = T.translation().x();
        pose_out[1] = T.translation().y();
        pose_out[2] = T.translation().z();
        pose_out[3] = q.x();
        pose_out[4] = q.y();
        pose_out[5] = q.z();
        pose_out[6] = q.w();
        return true;
    }

    /**
     * @brief 逆向运动学全部解析解 (导出多达 8 组解析解)
     */
    static bool computeAllIK(const opw_kinematics::Parameters<double>& params,
                            const std::vector<double>& pose,
                            std::vector<std::vector<double>>& all_solutions_out) {
        if (pose.size() < 7) return false;

        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.translation() = Eigen::Vector3d(pose[0], pose[1], pose[2]);
        Eigen::Quaterniond q(pose[6], pose[3], pose[4], pose[5]);
        // Convert URDF tool0 orientation to OPW flange orientation: R_opw = R_urdf * Ry(+pi/2)
        Eigen::Matrix3d R_opw = q.matrix() * Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY());
        T.linear() = R_opw;

        auto solutions = opw_kinematics::inverse(params, T);
        all_solutions_out.clear();

        for (auto& s : solutions) {
            if (opw_kinematics::isValid(s)) {
                opw_kinematics::harmonizeTowardZero(s);
                all_solutions_out.emplace_back(s.begin(), s.end());
            }
        }
        return !all_solutions_out.empty();
    }

    /**
     * @brief 逆向运动学最优解 (寻找与种子角/前一轨迹点位移欧氏距离最小的解，防止轴翻转)
     */
    static bool computeNearestIK(const opw_kinematics::Parameters<double>& params,
                                const std::vector<double>& pose,
                                const std::vector<double>& seed_joints,
                                std::vector<double>& best_joint_angles) {
        std::vector<std::vector<double>> all_solutions;
        if (!computeAllIK(params, pose, all_solutions)) {
            return false;
        }

        double min_dist_sq = std::numeric_limits<double>::max();
        size_t best_idx = 0;

        for (size_t i = 0; i < all_solutions.size(); ++i) {
            double dist_sq = 0.0;
            for (size_t j = 0; j < 6; ++j) {
                double diff = all_solutions[i][j] - (j < seed_joints.size() ? seed_joints[j] : 0.0);
                // 周期归一化到 [-pi, pi] 考虑多圈连续性
                while (diff > M_PI) diff -= 2.0 * M_PI;
                while (diff < -M_PI) diff += 2.0 * M_PI;
                dist_sq += diff * diff;
            }
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                best_idx = i;
            }
        }

        best_joint_angles = all_solutions[best_idx];
        return true;
    }
};

} // namespace robot_planner
