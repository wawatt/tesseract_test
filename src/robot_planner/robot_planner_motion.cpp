#include "planner_impl.h"

namespace robot_planner {

namespace {
struct TrajectoryBranch {
    std::vector<std::vector<double>> trajectory;
    double accumulated_cost = 0.0;
};

bool applyToppra(VampDynamicLoader& loader,
                 const std::vector<std::vector<double>>& waypoints,
                 JointTrajectory& trajectory_out,
                 double max_velocity_scaling,
                 double max_acceleration_scaling) {
    if (loader.parameterize(waypoints, trajectory_out,
                            max_velocity_scaling, max_acceleration_scaling, 0.01) &&
        !trajectory_out.empty()) {
        return true;
    }

    trajectory_out.clear();
    if (waypoints.empty()) return false;
    trajectory_out.positions = waypoints;
    const double dt = 0.02;
    trajectory_out.time_stamps.resize(waypoints.size());
    for (size_t i = 0; i < waypoints.size(); ++i) {
        trajectory_out.time_stamps[i] = static_cast<double>(i) * dt;
    }
    return true;
}
} // anonymous namespace

bool RobotPlanner::warmupRoadmap(double warmup_time) {
    if (!pimpl_->vampReady()) {
        pimpl_->setLastError(PlannerStatus::NOT_INITIALIZED, "VAMP backend is not loaded.");
        return false;
    }
    return pimpl_->vamp_loader_->warmupRoadmap(warmup_time);
}

bool RobotPlanner::planFreespace(const std::vector<double>& start_joints, 
                                 const std::vector<double>& target_joints, 
                                 JointTrajectory& trajectory_out,
                                 double max_velocity_scaling,
                                 double max_acceleration_scaling,
                                 double planning_time,
                                 double range,
                                 double safety_margin,
                                 double collision_coeff,
                                 const std::string& planner_type) {
    trajectory_out.clear();

    if (!pimpl_->env_) {
        pimpl_->setLastError(PlannerStatus::NOT_INITIALIZED, "Environment not initialized.");
        return false;
    }

    // Adaptive dispatch: if target has 7 elements and start has 6, user provided a Cartesian pose!
    if (target_joints.size() == 7 && start_joints.size() == 6) {
        return planFreespacePose(start_joints, target_joints, trajectory_out,
                                 max_velocity_scaling, max_acceleration_scaling,
                                 planning_time, range, safety_margin, collision_coeff,
                                 planner_type);
    }

    std::vector<std::string> joint_names = pimpl_->env_->getJointGroup(pimpl_->manipulator_name_)->getJointNames();
    if (start_joints.size() != joint_names.size() || target_joints.size() != joint_names.size()) {
        pimpl_->setLastError(PlannerStatus::INVALID_ARGUMENTS, "Joint vector size mismatch.");
        return false;
    }

    // Joint limit checks
    for (size_t j = 0; j < start_joints.size(); ++j) {
        if (j < pimpl_->joint_limits_min_.size()) {
            if (start_joints[j] < pimpl_->joint_limits_min_[j] - 1e-4 || start_joints[j] > pimpl_->joint_limits_max_[j] + 1e-4) {
                pimpl_->setLastError(PlannerStatus::JOINT_LIMIT_VIOLATED, "Start joints violate URDF physical limits.");
                return false;
            }
            if (target_joints[j] < pimpl_->joint_limits_min_[j] - 1e-4 || target_joints[j] > pimpl_->joint_limits_max_[j] + 1e-4) {
                pimpl_->setLastError(PlannerStatus::JOINT_LIMIT_VIOLATED, "Target joints violate URDF physical limits.");
                return false;
            }
        }
    }

    (void)collision_coeff;

    if (!pimpl_->vampReady() || start_joints.size() != 6) {
        pimpl_->setLastError(PlannerStatus::NOT_INITIALIZED, "VAMP backend is not loaded or joint vector is not 6-DOF.");
        return false;
    }

    auto t_v0 = std::chrono::high_resolution_clock::now();
    bool traj_ok = pimpl_->vamp_loader_->planTrajectory(start_joints, target_joints, trajectory_out,
                                                        max_velocity_scaling, max_acceleration_scaling,
                                                        planning_time, range, safety_margin, planner_type);
    auto t_v1 = std::chrono::high_resolution_clock::now();
    double vamp_ms = std::chrono::duration<double, std::milli>(t_v1 - t_v0).count();
    if (traj_ok && !trajectory_out.empty()) {
        std::cout << "[RobotPlanner] VAMP Pipeline (PRM -> B-Spline L-BFGS -> TOPP-RA ["
                  << vamp_ms << " ms]) SUCCESS! (Points: " << trajectory_out.positions.size() << ")" << std::endl;
        pimpl_->setLastError(PlannerStatus::SUCCESS, "");
        return true;
    }

    pimpl_->setLastError(PlannerStatus::PLANNING_TIMEOUT, "Freespace planning failed or timed out.");
    return false;
}

bool RobotPlanner::planFreespacePose(const std::vector<double>& start_joints, 
                                     const std::vector<double>& target_pose, 
                                     JointTrajectory& trajectory_out,
                                 double max_velocity_scaling,
                                 double max_acceleration_scaling,
                                 double planning_time,
                                 double range,
                                 double safety_margin,
                                 double collision_coeff,
                                 const std::string& planner_type,
                                 size_t max_seeds) {
    trajectory_out.clear();
    if (target_pose.size() < 7) {
        pimpl_->setLastError(PlannerStatus::INVALID_ARGUMENTS, "planFreespace: target_pose size must be at least 7.");
        return false;
    }

    // 1. Compute all collision-free OPW analytical IK seeds, sorted by proximity to start_joints
    std::vector<std::vector<double>> seeds;
    if (!computeAllCollisionFreeIK(target_pose, start_joints, seeds) || seeds.empty()) {
        return false;
    }

    if (seeds.size() > max_seeds) {
        seeds.resize(max_seeds);
    }

    // 2. Nearest seeds first with a shared wall-clock budget (cuRobo multi-seed).
    // Easy C-space connections return immediately via the VAMP direct-connect path.
    std::string first_failure_reason;
    double remaining = std::max(0.05, planning_time);
    for (size_t i = 0; i < seeds.size(); ++i) {
        if (remaining < 0.02) {
            break;
        }

        double seed_time;
        if (i == 0) {
            seed_time = std::min(remaining, std::max(0.08, planning_time * 0.40));
        } else if (i == 1) {
            seed_time = std::min(remaining, std::max(0.08, planning_time * 0.25));
        } else {
            seed_time = remaining / static_cast<double>(seeds.size() - i);
        }

        auto t0 = std::chrono::high_resolution_clock::now();
        JointTrajectory candidate_traj;
        bool ok = planFreespace(start_joints, seeds[i], candidate_traj,
                                max_velocity_scaling, max_acceleration_scaling,
                                seed_time, range, safety_margin, collision_coeff, planner_type);
        auto t1 = std::chrono::high_resolution_clock::now();
        remaining -= std::chrono::duration<double>(t1 - t0).count();

        if (ok && !candidate_traj.positions.empty()) {
            std::string reason;
            if (validateTrajectory(candidate_traj, nullptr, &reason)) {
                trajectory_out = std::move(candidate_traj);
                pimpl_->setLastError(PlannerStatus::SUCCESS, "");
                return true;
            }
        }
        if (first_failure_reason.empty()) {
            first_failure_reason = pimpl_->last_error_;
        }
    }

    pimpl_->setLastError(PlannerStatus::TRAJECTORY_FAILED, 
        "All " + std::to_string(seeds.size()) + " collision-free seeds failed planning/optimization. Initial error: " + first_failure_reason);
    return false;
}

bool RobotPlanner::planLinear(const std::vector<double>& start_joints, 
                              const std::vector<double>& target_pose, 
                              JointTrajectory& trajectory_out,
                              double max_velocity_scaling,
                              double max_acceleration_scaling,
                              double step_size,
                              double safety_margin,
                              double collision_coeff) {
    trajectory_out.clear();
    if (target_pose.size() < 7) {
        pimpl_->setLastError(PlannerStatus::INVALID_ARGUMENTS, "planLinear: target_pose size must be at least 7.");
        return false;
    }
    if (step_size <= 0.001) step_size = 0.005;

    if (!pimpl_->env_) {
        pimpl_->setLastError(PlannerStatus::NOT_INITIALIZED, "Environment not initialized.");
        return false;
    }
    std::vector<std::string> joint_names = pimpl_->env_->getJointGroup(pimpl_->manipulator_name_)->getJointNames();
    if (start_joints.size() != joint_names.size()) {
        pimpl_->setLastError(PlannerStatus::INVALID_ARGUMENTS, "start_joints size mismatch.");
        return false;
    }

    if (!pimpl_->vampReady() || start_joints.size() != 6) {
        pimpl_->setLastError(PlannerStatus::NOT_INITIALIZED, "VAMP backend is not loaded or joint vector is not 6-DOF.");
        return false;
    }
    {
        std::vector<double> start_pose;
        if (!computeFK(start_joints, start_pose)) {
            pimpl_->setLastError(PlannerStatus::INTERNAL_ERROR, "Failed to compute FK for start joints.");
            return false;
        }

        Eigen::Vector3d p_start(start_pose[0], start_pose[1], start_pose[2]);
        Eigen::Vector3d p_target(target_pose[0], target_pose[1], target_pose[2]);
        Eigen::Quaterniond q_start(start_pose[6], start_pose[3], start_pose[4], start_pose[5]);
        Eigen::Quaterniond q_target(target_pose[6], target_pose[3], target_pose[4], target_pose[5]);

        double dist = (p_target - p_start).norm();
        int steps = std::max(10, static_cast<int>(std::ceil(dist / step_size)));

        // Multi-candidate beam search branch tracking (cuRobo-style continuity protection)
        // Tracks up to BEAM_WIDTH=3 continuous IK branches to eliminate wrist flips (J4/J6 jumps of ~pi)
        const size_t BEAM_WIDTH = 3;
        std::vector<TrajectoryBranch> active_branches;
        active_branches.push_back({ { start_joints }, 0.0 });

        for (int k = 1; k <= steps; ++k) {
            double t = static_cast<double>(k) / steps;
            Eigen::Vector3d p_t = (1.0 - t) * p_start + t * p_target;
            Eigen::Quaterniond q_t = q_start.slerp(t, q_target);

            std::vector<double> step_pose = { p_t.x(), p_t.y(), p_t.z(), q_t.x(), q_t.y(), q_t.z(), q_t.w() };

            // 1. Get all in-limit analytical IK solutions (up to 8)
            std::vector<std::vector<double>> all_sols;
            if (!computeAllIK(step_pose, all_sols) || all_sols.empty()) {
                pimpl_->setLastError(PlannerStatus::IK_FAILED, 
                    "Linear IK failed at step " + std::to_string(k) + "/" + std::to_string(steps) + " (no analytical solution within URDF limits).");
                return false;
            }

            // 2. Filter out solutions in collision or with critical kinematic singularity
            std::vector<std::vector<double>> valid_sols;
            for (const auto& sol : all_sols) {
                if (checkCollision(sol)) continue;

                double w_score = 0.0;
                if (computeManipulability(sol, w_score) && w_score < 1e-4) {
                    continue;
                }
                valid_sols.push_back(sol);
            }

            if (valid_sols.empty()) {
                pimpl_->setLastError(PlannerStatus::COLLISION_DETECTED, 
                    "Linear path blocked at step " + std::to_string(k) + "/" + std::to_string(steps) + ": all candidate branches collide with obstacles or hit singularity.");
                return false;
            }

            // 3. Connect each active branch to all valid solutions without axis jumps
            std::vector<TrajectoryBranch> next_branches;
            for (const auto& branch : active_branches) {
                const auto& prev_joints = branch.trajectory.back();

                for (const auto& cand_joints : valid_sols) {
                    bool continuous = true;
                    double step_cost = 0.0;
                    for (size_t j = 0; j < cand_joints.size(); ++j) {
                        double diff = cand_joints[j] - prev_joints[j];
                        if (std::abs(diff) > 0.8) {
                            continuous = false;
                            break;
                        }
                        step_cost += diff * diff;
                    }

                    if (continuous) {
                        TrajectoryBranch new_branch = branch;
                        new_branch.trajectory.push_back(cand_joints);
                        new_branch.accumulated_cost += step_cost;
                        next_branches.push_back(std::move(new_branch));
                    }
                }
            }

            if (next_branches.empty()) {
                pimpl_->setLastError(PlannerStatus::SINGULARITY_DETECTED, 
                    "Linear path discontinuous at step " + std::to_string(k) + "/" + std::to_string(steps) + ": unavoidable axis flip/jump across all branches.");
                return false;
            }

            // 4. Sort next branches by accumulated_cost ascending and prune to BEAM_WIDTH
            std::stable_sort(next_branches.begin(), next_branches.end(),
                             [](const TrajectoryBranch& a, const TrajectoryBranch& b) {
                                 return a.accumulated_cost < b.accumulated_cost;
                             });

            if (next_branches.size() > BEAM_WIDTH) {
                next_branches.resize(BEAM_WIDTH);
            }
            active_branches = std::move(next_branches);
        }

        const auto& seed_traj = active_branches.front().trajectory;
        (void)collision_coeff;
        (void)safety_margin;

        if (!applyToppra(*pimpl_->vamp_loader_, seed_traj, trajectory_out, max_velocity_scaling, max_acceleration_scaling)) {
            pimpl_->setLastError(PlannerStatus::TRAJECTORY_FAILED, "Linear path TOPP-RA parameterization failed.");
            return false;
        }
        pimpl_->setLastError(PlannerStatus::SUCCESS, "");
        return true;
    }
}

bool RobotPlanner::planCircular(const std::vector<double>& start_joints, 
                                const std::vector<double>& aux_pose, 
                                const std::vector<double>& target_pose, 
                                JointTrajectory& trajectory_out,
                                double max_velocity_scaling,
                                double max_acceleration_scaling,
                                double step_size,
                                double safety_margin,
                                double collision_coeff) {
    trajectory_out.clear();
    if (aux_pose.size() < 7 || target_pose.size() < 7) {
        pimpl_->setLastError(PlannerStatus::INVALID_ARGUMENTS, "planCircular: pose size must be at least 7.");
        return false;
    }
    if (step_size <= 0.001) step_size = 0.005;

    if (!pimpl_->env_) {
        pimpl_->setLastError(PlannerStatus::NOT_INITIALIZED, "Environment not initialized.");
        return false;
    }
    std::vector<std::string> joint_names = pimpl_->env_->getJointGroup(pimpl_->manipulator_name_)->getJointNames();
    if (start_joints.size() != joint_names.size()) {
        pimpl_->setLastError(PlannerStatus::INVALID_ARGUMENTS, "planCircular: start_joints size mismatch.");
        return false;
    }

    if (!pimpl_->vampReady() || start_joints.size() != 6) {
        pimpl_->setLastError(PlannerStatus::NOT_INITIALIZED, "VAMP backend is not loaded or joint vector is not 6-DOF.");
        return false;
    }
    {
        std::vector<double> start_pose;
        if (!computeFK(start_joints, start_pose)) {
            pimpl_->setLastError(PlannerStatus::INTERNAL_ERROR, "Failed to compute FK for start joints.");
            return false;
        }

        Eigen::Vector3d p0(start_pose[0], start_pose[1], start_pose[2]);
        Eigen::Vector3d p1(aux_pose[0], aux_pose[1], aux_pose[2]);
        Eigen::Vector3d p2(target_pose[0], target_pose[1], target_pose[2]);

        Eigen::Vector3d v01 = p1 - p0;
        Eigen::Vector3d v02 = p2 - p0;
        Eigen::Vector3d normal = v01.cross(v02);
        if (normal.norm() < 1e-6) {
            pimpl_->setLastError(PlannerStatus::INVALID_ARGUMENTS, "planCircular: Points are collinear, cannot form a circle.");
            return false;
        }
        normal.normalize();

        Eigen::Vector3d m01 = 0.5 * (p0 + p1);
        Eigen::Vector3d m12 = 0.5 * (p1 + p2);
        Eigen::Vector3d d01 = normal.cross(v01).normalized();
        Eigen::Vector3d v12 = p2 - p1;
        Eigen::Vector3d d12 = normal.cross(v12).normalized();

        Eigen::Matrix2d A;
        A << d01.x(), -d12.x(),
             d01.y(), -d12.y();
        Eigen::Vector2d b_vec(m12.x() - m01.x(), m12.y() - m01.y());
        if (std::abs(A.determinant()) < 1e-6) {
            A << d01.x(), -d12.x(),
                 d01.z(), -d12.z();
            b_vec << m12.x() - m01.x(), m12.z() - m01.z();
        }
        Eigen::Vector2d uv = A.colPivHouseholderQr().solve(b_vec);
        Eigen::Vector3d center = m01 + uv(0) * d01;
        double radius = (p0 - center).norm();

        Eigen::Vector3d u_vec = (p0 - center).normalized();
        Eigen::Vector3d w_vec = normal.cross(u_vec).normalized();

        double theta1 = std::atan2((p1 - center).dot(w_vec), (p1 - center).dot(u_vec));
        if (theta1 < 0) theta1 += 2.0 * M_PI;
        double theta2 = std::atan2((p2 - center).dot(w_vec), (p2 - center).dot(u_vec));
        if (theta2 < theta1) theta2 += 2.0 * M_PI;

        double arc_length = radius * theta2;
        int steps = std::max(15, static_cast<int>(std::ceil(arc_length / step_size)));

        Eigen::Quaterniond q0(start_pose[6], start_pose[3], start_pose[4], start_pose[5]);
        Eigen::Quaterniond q1(aux_pose[6], aux_pose[3], aux_pose[4], aux_pose[5]);
        Eigen::Quaterniond q2(target_pose[6], target_pose[3], target_pose[4], target_pose[5]);

        // Multi-candidate beam search branch tracking (cuRobo-style continuity protection)
        const size_t BEAM_WIDTH = 3;
        std::vector<TrajectoryBranch> active_branches;
        active_branches.push_back({ { start_joints }, 0.0 });

        for (int k = 1; k <= steps; ++k) {
            double t = static_cast<double>(k) / steps;
            double angle = t * theta2;
            Eigen::Vector3d p_t = center + radius * (std::cos(angle) * u_vec + std::sin(angle) * w_vec);

            Eigen::Quaterniond q_t;
            if (t <= 0.5) {
                q_t = q0.slerp(t * 2.0, q1);
            } else {
                q_t = q1.slerp((t - 0.5) * 2.0, q2);
            }

            std::vector<double> step_pose = { p_t.x(), p_t.y(), p_t.z(), q_t.x(), q_t.y(), q_t.z(), q_t.w() };

            // 1. Get all in-limit analytical IK solutions (up to 8)
            std::vector<std::vector<double>> all_sols;
            if (!computeAllIK(step_pose, all_sols) || all_sols.empty()) {
                pimpl_->setLastError(PlannerStatus::IK_FAILED, 
                    "Circular IK failed at step " + std::to_string(k) + "/" + std::to_string(steps) + " (no analytical solution within URDF limits).");
                return false;
            }

            // 2. Filter out solutions in collision or with critical kinematic singularity
            std::vector<std::vector<double>> valid_sols;
            for (const auto& sol : all_sols) {
                if (checkCollision(sol)) continue;

                double w_score = 0.0;
                if (computeManipulability(sol, w_score) && w_score < 1e-4) {
                    continue;
                }
                valid_sols.push_back(sol);
            }

            if (valid_sols.empty()) {
                pimpl_->setLastError(PlannerStatus::COLLISION_DETECTED, 
                    "Circular path blocked at step " + std::to_string(k) + "/" + std::to_string(steps) + ": all candidate branches collide with obstacles or hit singularity.");
                return false;
            }

            // 3. Connect each active branch to all valid solutions without axis jumps
            std::vector<TrajectoryBranch> next_branches;
            for (const auto& branch : active_branches) {
                const auto& prev_joints = branch.trajectory.back();

                for (const auto& cand_joints : valid_sols) {
                    bool continuous = true;
                    double step_cost = 0.0;
                    for (size_t j = 0; j < cand_joints.size(); ++j) {
                        double diff = cand_joints[j] - prev_joints[j];
                        if (std::abs(diff) > 0.8) {
                            continuous = false;
                            break;
                        }
                        step_cost += diff * diff;
                    }

                    if (continuous) {
                        TrajectoryBranch new_branch = branch;
                        new_branch.trajectory.push_back(cand_joints);
                        new_branch.accumulated_cost += step_cost;
                        next_branches.push_back(std::move(new_branch));
                    }
                }
            }

            if (next_branches.empty()) {
                pimpl_->setLastError(PlannerStatus::SINGULARITY_DETECTED, 
                    "Circular path discontinuous at step " + std::to_string(k) + "/" + std::to_string(steps) + ": unavoidable axis flip/jump across all branches.");
                return false;
            }

            // 4. Sort next branches by accumulated_cost ascending and prune to BEAM_WIDTH
            std::stable_sort(next_branches.begin(), next_branches.end(),
                             [](const TrajectoryBranch& a, const TrajectoryBranch& b) {
                                 return a.accumulated_cost < b.accumulated_cost;
                             });

            if (next_branches.size() > BEAM_WIDTH) {
                next_branches.resize(BEAM_WIDTH);
            }
            active_branches = std::move(next_branches);
        }

        const auto& seed_traj = active_branches.front().trajectory;
        (void)collision_coeff;
        (void)safety_margin;

        if (!applyToppra(*pimpl_->vamp_loader_, seed_traj, trajectory_out, max_velocity_scaling, max_acceleration_scaling)) {
            pimpl_->setLastError(PlannerStatus::TRAJECTORY_FAILED, "Circular path TOPP-RA parameterization failed.");
            return false;
        }
        pimpl_->setLastError(PlannerStatus::SUCCESS, "");
        return true;
    }
}

} // namespace robot_planner
