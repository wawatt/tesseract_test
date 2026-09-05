#include "planner_impl.h"

namespace robot_planner {

namespace {
struct TrajectoryBranch {
    std::vector<std::vector<double>> trajectory;
    double accumulated_cost = 0.0;
};
} // anonymous namespace

bool RobotPlanner::warmupRoadmap(double warmup_time) {
    if (pimpl_->backend_ == PlannerBackend::VAMP && pimpl_->vamp_loader_ && pimpl_->vamp_loader_->isLoaded()) {
        return pimpl_->vamp_loader_->warmupRoadmap(warmup_time);
    }
    return true;
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

    auto t_pipe_0 = std::chrono::high_resolution_clock::now();
    double vamp_ms = 0.0;

    // 步骤 1: OMPL(vamp) 全局避障寻路
    std::vector<std::vector<double>> seed_trajectory;
    bool vamp_ok = false;
    if (pimpl_->backend_ == PlannerBackend::VAMP && pimpl_->vamp_loader_ && start_joints.size() == 6) {
        auto t_v0 = std::chrono::high_resolution_clock::now();
        vamp_ok = pimpl_->vamp_loader_->planFreespace(start_joints, target_joints, seed_trajectory,
                                                      planning_time, range, safety_margin, planner_type);
        auto t_v1 = std::chrono::high_resolution_clock::now();
        vamp_ms = std::chrono::duration<double, std::milli>(t_v1 - t_v0).count();
    }

    // 步骤 2 & 3: TrajOpt smoothing -> Time Parameterization
    tesseract::common::ManipulatorInfo manip_info(pimpl_->manipulator_name_, pimpl_->base_link_, pimpl_->tool_link_);
    tesseract::command_language::CompositeInstruction program("FREESPACE_PIPELINE", manip_info);

    auto profiles = std::make_shared<tesseract::common::ProfileDictionary>();

    auto trajopt_composite_profile = std::make_shared<tesseract::motion_planners::TrajOptDefaultCompositeProfile>();
    trajopt_composite_profile->collision_cost_config = trajopt_common::TrajOptCollisionConfig(safety_margin, collision_coeff);
    trajopt_composite_profile->collision_cost_config.enabled = true;
    profiles->addProfile("TrajOptMotionPlannerTask", "FREESPACE_PIPELINE", trajopt_composite_profile);

    auto trajopt_move_profile = std::make_shared<tesseract::motion_planners::TrajOptDefaultMoveProfile>();
    profiles->addProfile("TrajOptMotionPlannerTask", "FREESPACE", trajopt_move_profile);

    std::string pipeline_name = "FreespacePipeline";

    if (vamp_ok && !seed_trajectory.empty()) {
        pipeline_name = "TrajOptPipeline";

        for (size_t i = 0; i < seed_trajectory.size(); ++i) {
            Eigen::VectorXd pt = Eigen::Map<const Eigen::VectorXd>(seed_trajectory[i].data(), seed_trajectory[i].size());
            tesseract::command_language::StateWaypoint wp(joint_names, pt);
            tesseract::command_language::MoveInstruction inst(
                wp, tesseract::command_language::MoveInstructionType::FREESPACE, "FREESPACE");
            if (i == 0) inst.setDescription("Start");
            program.push_back(inst);
        }
    } else {
        auto ompl_profile = std::make_shared<tesseract::motion_planners::OMPLRealVectorMoveProfile>();
        std::shared_ptr<tesseract::motion_planners::OMPLPlannerConfigurator> ompl_planner_config;
        if (planner_type == "RRTstar" || planner_type == "RRT*" || planner_type == "rrtstar") {
            auto rrtstar = std::make_shared<tesseract::motion_planners::RRTstarConfigurator>();
            rrtstar->range = range;
            ompl_planner_config = rrtstar;
        } else if (planner_type == "RRTConnect" || planner_type == "rrtconnect") {
            auto rrtconnect = std::make_shared<tesseract::motion_planners::RRTConnectConfigurator>();
            rrtconnect->range = range;
            ompl_planner_config = rrtconnect;
        } else {
            ompl_planner_config = std::make_shared<tesseract::motion_planners::PRMConfigurator>();
        }
        ompl_profile->solver_config.planning_time = planning_time;
        ompl_profile->solver_config.planners = { ompl_planner_config, ompl_planner_config };
        profiles->addProfile("OMPLMotionPlannerTask", "FREESPACE", ompl_profile);

        Eigen::VectorXd start = Eigen::Map<const Eigen::VectorXd>(start_joints.data(), start_joints.size());
        Eigen::VectorXd target = Eigen::Map<const Eigen::VectorXd>(target_joints.data(), target_joints.size());
        tesseract::command_language::StateWaypoint wp0(joint_names, start);
        tesseract::command_language::MoveInstruction start_inst(wp0, tesseract::command_language::MoveInstructionType::FREESPACE, "FREESPACE");
        tesseract::command_language::StateWaypoint wp1(joint_names, target);
        tesseract::command_language::MoveInstruction plan_inst(wp1, tesseract::command_language::MoveInstructionType::FREESPACE, "FREESPACE");
        program.push_back(start_inst);
        program.push_back(plan_inst);
    }

    if (pimpl_->factory_) {
        auto task = pimpl_->factory_->createTaskComposerNode(pipeline_name);
        if (task) {
            auto executor = pimpl_->factory_->createTaskComposerExecutor("TaskflowExecutor");
            auto data = std::make_unique<tesseract::task_composer::TaskComposerDataStorage>();
            data->setData("planning_input", program);
            data->setData("environment", std::shared_ptr<const tesseract::environment::Environment>(pimpl_->env_));
            data->setData("profiles", profiles);

            auto context = std::make_shared<tesseract::task_composer::TaskComposerContext>(task->getName(), std::move(data));
            auto t_opt_0 = std::chrono::high_resolution_clock::now();
            auto future = executor->run(*task, std::move(context));
            future->wait();
            auto t_opt_1 = std::chrono::high_resolution_clock::now();
            double opt_ms = std::chrono::duration<double, std::milli>(t_opt_1 - t_opt_0).count();
            auto t_pipe_1 = std::chrono::high_resolution_clock::now();
            double total_ms = std::chrono::duration<double, std::milli>(t_pipe_1 - t_pipe_0).count();

            if (future->context->isSuccessful()) {
                auto ci = future->context->data_storage->getData(task->getOutputKeys().get("program")).as<tesseract::command_language::CompositeInstruction>();
                auto traj = tesseract::command_language::toJointTrajectory(ci);

                for (const auto& wp : traj) {
                    trajectory_out.positions.emplace_back(wp.position.data(), wp.position.data() + wp.position.size());
                    if (wp.velocity.size() == wp.position.size()) {
                        trajectory_out.velocities.emplace_back(wp.velocity.data(), wp.velocity.data() + wp.velocity.size());
                    }
                    if (wp.acceleration.size() == wp.position.size()) {
                        trajectory_out.accelerations.emplace_back(wp.acceleration.data(), wp.acceleration.data() + wp.acceleration.size());
                    }
                    trajectory_out.time_stamps.push_back(wp.time);
                }

                // Apply velocity/acceleration scaling
                if (max_velocity_scaling > 0.0 && max_velocity_scaling <= 1.0 && max_velocity_scaling != 1.0) {
                    double time_factor = 1.0 / max_velocity_scaling;
                    for (size_t i = 0; i < trajectory_out.time_stamps.size(); ++i) {
                        trajectory_out.time_stamps[i] *= time_factor;
                        for (size_t j = 0; j < trajectory_out.velocities[i].size(); ++j) {
                            trajectory_out.velocities[i][j] *= max_velocity_scaling;
                        }
                    }
                }
                if (max_acceleration_scaling > 0.0 && max_acceleration_scaling <= 1.0 && max_acceleration_scaling != 1.0) {
                    for (size_t i = 0; i < trajectory_out.accelerations.size(); ++i) {
                        for (size_t j = 0; j < trajectory_out.accelerations[i].size(); ++j) {
                            trajectory_out.accelerations[i][j] *= max_acceleration_scaling;
                        }
                    }
                }

                if (vamp_ok && !seed_trajectory.empty()) {
                    std::cout << "[RobotPlanner] VAMP Hybrid Pipeline (OMPL(vamp) SIMD [" << vamp_ms 
                              << " ms] -> TrajOpt smoothing + TimeParam [" << opt_ms << " ms] -> Total: " 
                              << total_ms << " ms) SUCCESS! (Points: " << trajectory_out.positions.size() << ")" << std::endl;
                } else {
                    std::cout << "[RobotPlanner] Tesseract Pipeline (Bullet OMPL -> TrajOpt smoothing -> TimeParam, Total: " 
                              << total_ms << " ms) SUCCESS! (Points: " << trajectory_out.positions.size() << ")" << std::endl;
                }
                pimpl_->setLastError(PlannerStatus::SUCCESS, "");
                return true;
            }
        }
    }

    if (vamp_ok && !seed_trajectory.empty()) {
        trajectory_out.positions = seed_trajectory;
        double cur_t = 0.0;
        for (size_t i = 0; i < seed_trajectory.size(); ++i) {
            trajectory_out.time_stamps.push_back(cur_t);
            cur_t += 0.05;
        }
        pimpl_->setLastError(PlannerStatus::SUCCESS, "TrajOpt smoothing fallback, returned OMPL(vamp) trajectory.");
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

    // 2. Try optimizing with each candidate seed (cuRobo multi-seed paradigm)
    // Avoids TrajOpt getting stuck in a bad local minimum / basin of a single seed!
    std::string first_failure_reason;
    for (size_t i = 0; i < seeds.size(); ++i) {
        JointTrajectory candidate_traj;
        bool ok = planFreespace(start_joints, seeds[i], candidate_traj,
                                max_velocity_scaling, max_acceleration_scaling,
                                planning_time, range, safety_margin, collision_coeff, planner_type);
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

    // VAMP accelerated mode (OPW IK + SIMD collision check + TrajOpt parameterization)
    if (pimpl_->backend_ == PlannerBackend::VAMP && pimpl_->vamp_loader_ && start_joints.size() == 6) {
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

        // Time parameterization via TrajOpt pipeline
        if (pimpl_->factory_) {
            tesseract::common::ManipulatorInfo manip_info(pimpl_->manipulator_name_, pimpl_->base_link_, pimpl_->tool_link_);
            tesseract::command_language::CompositeInstruction program("LIN_TRAJOPT", manip_info);
            auto profiles = std::make_shared<tesseract::common::ProfileDictionary>();

            auto composite_profile = std::make_shared<tesseract::motion_planners::TrajOptDefaultCompositeProfile>();
            composite_profile->collision_cost_config = trajopt_common::TrajOptCollisionConfig(safety_margin, collision_coeff);
            composite_profile->collision_cost_config.enabled = true;
            profiles->addProfile("TrajOptMotionPlannerTask", "LIN_TRAJOPT", composite_profile);

            auto move_profile = std::make_shared<tesseract::motion_planners::TrajOptDefaultMoveProfile>();
            profiles->addProfile("TrajOptMotionPlannerTask", "FREESPACE", move_profile);

            for (size_t i = 0; i < seed_traj.size(); ++i) {
                Eigen::VectorXd pt = Eigen::Map<const Eigen::VectorXd>(seed_traj[i].data(), seed_traj[i].size());
                tesseract::command_language::StateWaypoint wp(joint_names, pt);
                tesseract::command_language::MoveInstruction inst(wp, tesseract::command_language::MoveInstructionType::FREESPACE, "FREESPACE");
                if (i == 0) inst.setDescription("Start");
                program.push_back(inst);
            }

            auto task = pimpl_->factory_->createTaskComposerNode("TrajOptPipeline");
            if (task) {
                auto executor = pimpl_->factory_->createTaskComposerExecutor("TaskflowExecutor");
                auto data = std::make_unique<tesseract::task_composer::TaskComposerDataStorage>();
                data->setData("planning_input", program);
                data->setData("environment", std::shared_ptr<const tesseract::environment::Environment>(pimpl_->env_));
                data->setData("profiles", profiles);

                auto context = std::make_shared<tesseract::task_composer::TaskComposerContext>(task->getName(), std::move(data));
                auto future = executor->run(*task, std::move(context));
                future->wait();

                if (future->context->isSuccessful()) {
                    auto ci = future->context->data_storage->getData(task->getOutputKeys().get("program")).as<tesseract::command_language::CompositeInstruction>();
                    auto traj = tesseract::command_language::toJointTrajectory(ci);

                    for (const auto& wp : traj) {
                        trajectory_out.positions.emplace_back(wp.position.data(), wp.position.data() + wp.position.size());
                        if (wp.velocity.size() == wp.position.size()) {
                            trajectory_out.velocities.emplace_back(wp.velocity.data(), wp.velocity.data() + wp.velocity.size());
                        }
                        if (wp.acceleration.size() == wp.position.size()) {
                            trajectory_out.accelerations.emplace_back(wp.acceleration.data(), wp.acceleration.data() + wp.acceleration.size());
                        }
                        trajectory_out.time_stamps.push_back(wp.time);
                    }

                    if (max_velocity_scaling > 0.0 && max_velocity_scaling <= 1.0 && max_velocity_scaling != 1.0) {
                        double time_factor = 1.0 / max_velocity_scaling;
                        for (size_t i = 0; i < trajectory_out.time_stamps.size(); ++i) {
                            trajectory_out.time_stamps[i] *= time_factor;
                            for (size_t j = 0; j < trajectory_out.velocities[i].size(); ++j) {
                                trajectory_out.velocities[i][j] *= max_velocity_scaling;
                            }
                        }
                    }
                    if (max_acceleration_scaling > 0.0 && max_acceleration_scaling <= 1.0 && max_acceleration_scaling != 1.0) {
                        for (size_t i = 0; i < trajectory_out.accelerations.size(); ++i) {
                            for (size_t j = 0; j < trajectory_out.accelerations[i].size(); ++j) {
                                trajectory_out.accelerations[i][j] *= max_acceleration_scaling;
                            }
                        }
                    }
                    pimpl_->setLastError(PlannerStatus::SUCCESS, "");
                    return true;
                }
            }
        }

        // Fallback: populate basic timestamps
        trajectory_out.positions = seed_traj;
        double cur_t = 0.0;
        for (size_t i = 0; i < seed_traj.size(); ++i) {
            trajectory_out.time_stamps.push_back(cur_t);
            cur_t += 0.05;
        }
        pimpl_->setLastError(PlannerStatus::SUCCESS, "");
        return true;
    }

    // Native Tesseract TrajOpt mode
    if (!pimpl_->factory_) {
        pimpl_->setLastError(PlannerStatus::NOT_INITIALIZED, "Task composer factory not initialized.");
        return false;
    }

    tesseract::common::ManipulatorInfo manip_info(pimpl_->manipulator_name_, pimpl_->base_link_, pimpl_->tool_link_);
    tesseract::command_language::CompositeInstruction program("LIN_program", manip_info);
    
    Eigen::VectorXd start = Eigen::Map<const Eigen::VectorXd>(start_joints.data(), start_joints.size());
    tesseract::command_language::StateWaypoint wp0(joint_names, start);
    tesseract::command_language::MoveInstruction start_inst(wp0, tesseract::command_language::MoveInstructionType::FREESPACE, "freespace_profile");
    
    Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
    target.translation() = Eigen::Vector3d(target_pose[0], target_pose[1], target_pose[2]);
    Eigen::Quaterniond q(target_pose[6], target_pose[3], target_pose[4], target_pose[5]);
    target.linear() = q.matrix();
    
    tesseract::command_language::CartesianWaypoint wp1(target);
    tesseract::command_language::MoveInstruction plan_inst(wp1, tesseract::command_language::MoveInstructionType::LINEAR, "RASTER");
    
    program.push_back(start_inst);
    program.push_back(plan_inst);
    
    auto profiles = std::make_shared<tesseract::common::ProfileDictionary>();
    auto composite_profile = std::make_shared<tesseract::motion_planners::TrajOptDefaultCompositeProfile>();
    composite_profile->collision_cost_config = trajopt_common::TrajOptCollisionConfig(safety_margin, collision_coeff);
    composite_profile->collision_cost_config.enabled = true;
    profiles->addProfile("TrajOptMotionPlannerTask", "LIN_program", composite_profile);
    
    auto move_profile = std::make_shared<tesseract::motion_planners::TrajOptDefaultMoveProfile>();
    profiles->addProfile("TrajOptMotionPlannerTask", "freespace_profile", move_profile);
    
    auto cart_profile = std::make_shared<tesseract::motion_planners::TrajOptDefaultMoveProfile>();
    cart_profile->cartesian_cost_config.enabled = false;
    cart_profile->cartesian_constraint_config.enabled = true;
    profiles->addProfile("TrajOptMotionPlannerTask", "RASTER", cart_profile);
    
    auto task = pimpl_->factory_->createTaskComposerNode("TrajOptPipeline");
    if (!task) {
        pimpl_->setLastError(PlannerStatus::INTERNAL_ERROR, "Failed to create TrajOptPipeline task node.");
        return false;
    }
    
    auto executor = pimpl_->factory_->createTaskComposerExecutor("TaskflowExecutor");
    auto data = std::make_unique<tesseract::task_composer::TaskComposerDataStorage>();
    data->setData("planning_input", program);
    data->setData("environment", std::shared_ptr<const tesseract::environment::Environment>(pimpl_->env_));
    data->setData("profiles", profiles);
    
    auto context = std::make_shared<tesseract::task_composer::TaskComposerContext>(task->getName(), std::move(data));
    auto future = executor->run(*task, std::move(context));
    future->wait();
    
    if (future->context->isSuccessful()) {
        auto ci = future->context->data_storage->getData(task->getOutputKeys().get("program")).as<tesseract::command_language::CompositeInstruction>();
        auto traj = tesseract::command_language::toJointTrajectory(ci);
        
        for (const auto& wp : traj) {
            trajectory_out.positions.emplace_back(wp.position.data(), wp.position.data() + wp.position.size());
            if (wp.velocity.size() == wp.position.size()) {
                trajectory_out.velocities.emplace_back(wp.velocity.data(), wp.velocity.data() + wp.velocity.size());
            }
            if (wp.acceleration.size() == wp.position.size()) {
                trajectory_out.accelerations.emplace_back(wp.acceleration.data(), wp.acceleration.data() + wp.acceleration.size());
            }
            trajectory_out.time_stamps.push_back(wp.time);
        }

        if (max_velocity_scaling > 0.0 && max_velocity_scaling <= 1.0 && max_velocity_scaling != 1.0) {
            double time_factor = 1.0 / max_velocity_scaling;
            for (size_t i = 0; i < trajectory_out.time_stamps.size(); ++i) {
                trajectory_out.time_stamps[i] *= time_factor;
                for (size_t j = 0; j < trajectory_out.velocities[i].size(); ++j) {
                    trajectory_out.velocities[i][j] *= max_velocity_scaling;
                }
            }
        }
        if (max_acceleration_scaling > 0.0 && max_acceleration_scaling <= 1.0 && max_acceleration_scaling != 1.0) {
            for (size_t i = 0; i < trajectory_out.accelerations.size(); ++i) {
                for (size_t j = 0; j < trajectory_out.accelerations[i].size(); ++j) {
                    trajectory_out.accelerations[i][j] *= max_acceleration_scaling;
                }
            }
        }
        pimpl_->setLastError(PlannerStatus::SUCCESS, "");
        return true;
    }

    pimpl_->setLastError(PlannerStatus::TRAJECTORY_FAILED, "Native linear trajectory optimization failed.");
    return false;
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

    // Analytical circular trajectory mode (OPW IK + collision check + TrajOpt parameterization)
    // Tesseract 0.35 SimplePlanner lacks native MoveInstructionType::CIRCULAR support, so analytical interpolation serves all 6-DOF arms
    if (start_joints.size() == 6) {
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

        // Time parameterization via TrajOpt pipeline
        if (pimpl_->factory_) {
            tesseract::common::ManipulatorInfo manip_info(pimpl_->manipulator_name_, pimpl_->base_link_, pimpl_->tool_link_);
            tesseract::command_language::CompositeInstruction program("CIRC_TRAJOPT", manip_info);
            auto profiles = std::make_shared<tesseract::common::ProfileDictionary>();

            auto composite_profile = std::make_shared<tesseract::motion_planners::TrajOptDefaultCompositeProfile>();
            composite_profile->collision_cost_config = trajopt_common::TrajOptCollisionConfig(safety_margin, collision_coeff);
            composite_profile->collision_cost_config.enabled = true;
            profiles->addProfile("TrajOptMotionPlannerTask", "CIRC_TRAJOPT", composite_profile);

            auto move_profile = std::make_shared<tesseract::motion_planners::TrajOptDefaultMoveProfile>();
            profiles->addProfile("TrajOptMotionPlannerTask", "FREESPACE", move_profile);

            for (size_t i = 0; i < seed_traj.size(); ++i) {
                Eigen::VectorXd pt = Eigen::Map<const Eigen::VectorXd>(seed_traj[i].data(), seed_traj[i].size());
                tesseract::command_language::StateWaypoint wp(joint_names, pt);
                tesseract::command_language::MoveInstruction inst(wp, tesseract::command_language::MoveInstructionType::FREESPACE, "FREESPACE");
                if (i == 0) inst.setDescription("Start");
                program.push_back(inst);
            }

            auto task = pimpl_->factory_->createTaskComposerNode("TrajOptPipeline");
            if (task) {
                auto executor = pimpl_->factory_->createTaskComposerExecutor("TaskflowExecutor");
                auto data = std::make_unique<tesseract::task_composer::TaskComposerDataStorage>();
                data->setData("planning_input", program);
                data->setData("environment", std::shared_ptr<const tesseract::environment::Environment>(pimpl_->env_));
                data->setData("profiles", profiles);

                auto context = std::make_shared<tesseract::task_composer::TaskComposerContext>(task->getName(), std::move(data));
                auto future = executor->run(*task, std::move(context));
                future->wait();

                if (future->context->isSuccessful()) {
                    auto ci = future->context->data_storage->getData(task->getOutputKeys().get("program")).as<tesseract::command_language::CompositeInstruction>();
                    auto traj = tesseract::command_language::toJointTrajectory(ci);

                    for (const auto& wp : traj) {
                        trajectory_out.positions.emplace_back(wp.position.data(), wp.position.data() + wp.position.size());
                        if (wp.velocity.size() == wp.position.size()) {
                            trajectory_out.velocities.emplace_back(wp.velocity.data(), wp.velocity.data() + wp.velocity.size());
                        }
                        if (wp.acceleration.size() == wp.position.size()) {
                            trajectory_out.accelerations.emplace_back(wp.acceleration.data(), wp.acceleration.data() + wp.acceleration.size());
                        }
                        trajectory_out.time_stamps.push_back(wp.time);
                    }

                    if (max_velocity_scaling > 0.0 && max_velocity_scaling <= 1.0 && max_velocity_scaling != 1.0) {
                        double time_factor = 1.0 / max_velocity_scaling;
                        for (size_t i = 0; i < trajectory_out.time_stamps.size(); ++i) {
                            trajectory_out.time_stamps[i] *= time_factor;
                            for (size_t j = 0; j < trajectory_out.velocities[i].size(); ++j) {
                                trajectory_out.velocities[i][j] *= max_velocity_scaling;
                            }
                        }
                    }
                    if (max_acceleration_scaling > 0.0 && max_acceleration_scaling <= 1.0 && max_acceleration_scaling != 1.0) {
                        for (size_t i = 0; i < trajectory_out.accelerations.size(); ++i) {
                            for (size_t j = 0; j < trajectory_out.accelerations[i].size(); ++j) {
                                trajectory_out.accelerations[i][j] *= max_acceleration_scaling;
                            }
                        }
                    }
                    pimpl_->setLastError(PlannerStatus::SUCCESS, "");
                    return true;
                }
            }
        }

        // Fallback
        trajectory_out.positions = seed_traj;
        double cur_t = 0.0;
        for (size_t i = 0; i < seed_traj.size(); ++i) {
            trajectory_out.time_stamps.push_back(cur_t);
            cur_t += 0.05;
        }
        pimpl_->setLastError(PlannerStatus::SUCCESS, "");
        return true;
    }

    // Native Tesseract mode
    if (!pimpl_->factory_) {
        pimpl_->setLastError(PlannerStatus::NOT_INITIALIZED, "Task composer factory not initialized.");
        return false;
    }
    
    tesseract::common::ManipulatorInfo manip_info(pimpl_->manipulator_name_, pimpl_->base_link_, pimpl_->tool_link_);
    tesseract::command_language::CompositeInstruction program("CIRC_program", manip_info);
    
    Eigen::VectorXd start = Eigen::Map<const Eigen::VectorXd>(start_joints.data(), start_joints.size());
    tesseract::command_language::StateWaypoint wp0(joint_names, start);
    tesseract::command_language::MoveInstruction start_inst(wp0, tesseract::command_language::MoveInstructionType::FREESPACE, "freespace_profile");
    
    Eigen::Isometry3d aux = Eigen::Isometry3d::Identity();
    aux.translation() = Eigen::Vector3d(aux_pose[0], aux_pose[1], aux_pose[2]);
    Eigen::Quaterniond q_aux(aux_pose[6], aux_pose[3], aux_pose[4], aux_pose[5]);
    aux.linear() = q_aux.matrix();
    tesseract::command_language::CartesianWaypoint wp_aux(aux);
    tesseract::command_language::MoveInstruction aux_inst(wp_aux, tesseract::command_language::MoveInstructionType::CIRCULAR, "RASTER");
    
    Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
    target.translation() = Eigen::Vector3d(target_pose[0], target_pose[1], target_pose[2]);
    Eigen::Quaterniond q_target(target_pose[6], target_pose[3], target_pose[4], target_pose[5]);
    target.linear() = q_target.matrix();
    tesseract::command_language::CartesianWaypoint wp_target(target);
    tesseract::command_language::MoveInstruction target_inst(wp_target, tesseract::command_language::MoveInstructionType::CIRCULAR, "RASTER");
    
    program.push_back(start_inst);
    program.push_back(aux_inst);
    program.push_back(target_inst);
    
    auto profiles = std::make_shared<tesseract::common::ProfileDictionary>();
    auto composite_profile = std::make_shared<tesseract::motion_planners::TrajOptDefaultCompositeProfile>();
    composite_profile->collision_cost_config = trajopt_common::TrajOptCollisionConfig(safety_margin, collision_coeff);
    composite_profile->collision_cost_config.enabled = true;
    profiles->addProfile("TrajOptMotionPlannerTask", "CIRC_program", composite_profile);
    
    auto move_profile = std::make_shared<tesseract::motion_planners::TrajOptDefaultMoveProfile>();
    profiles->addProfile("TrajOptMotionPlannerTask", "freespace_profile", move_profile);
    
    auto cart_profile = std::make_shared<tesseract::motion_planners::TrajOptDefaultMoveProfile>();
    cart_profile->cartesian_cost_config.enabled = false;
    cart_profile->cartesian_constraint_config.enabled = true;
    profiles->addProfile("TrajOptMotionPlannerTask", "RASTER", cart_profile);
    
    auto task = pimpl_->factory_->createTaskComposerNode("TrajOptPipeline");
    if (!task) {
        pimpl_->setLastError(PlannerStatus::INTERNAL_ERROR, "Failed to create TrajOptPipeline node.");
        return false;
    }
    
    auto executor = pimpl_->factory_->createTaskComposerExecutor("TaskflowExecutor");
    auto data = std::make_unique<tesseract::task_composer::TaskComposerDataStorage>();
    data->setData("planning_input", program);
    data->setData("environment", std::shared_ptr<const tesseract::environment::Environment>(pimpl_->env_));
    data->setData("profiles", profiles);
    
    auto context = std::make_shared<tesseract::task_composer::TaskComposerContext>(task->getName(), std::move(data));
    auto future = executor->run(*task, std::move(context));
    future->wait();
    
    if (future->context->isSuccessful()) {
        auto ci = future->context->data_storage->getData(task->getOutputKeys().get("program")).as<tesseract::command_language::CompositeInstruction>();
        auto traj = tesseract::command_language::toJointTrajectory(ci);
        
        for (const auto& wp : traj) {
            trajectory_out.positions.emplace_back(wp.position.data(), wp.position.data() + wp.position.size());
            if (wp.velocity.size() == wp.position.size()) {
                trajectory_out.velocities.emplace_back(wp.velocity.data(), wp.velocity.data() + wp.velocity.size());
            }
            if (wp.acceleration.size() == wp.position.size()) {
                trajectory_out.accelerations.emplace_back(wp.acceleration.data(), wp.acceleration.data() + wp.acceleration.size());
            }
            trajectory_out.time_stamps.push_back(wp.time);
        }

        if (max_velocity_scaling > 0.0 && max_velocity_scaling <= 1.0 && max_velocity_scaling != 1.0) {
            double time_factor = 1.0 / max_velocity_scaling;
            for (size_t i = 0; i < trajectory_out.time_stamps.size(); ++i) {
                trajectory_out.time_stamps[i] *= time_factor;
                for (size_t j = 0; j < trajectory_out.velocities[i].size(); ++j) {
                    trajectory_out.velocities[i][j] *= max_velocity_scaling;
                }
            }
        }
        if (max_acceleration_scaling > 0.0 && max_acceleration_scaling <= 1.0 && max_acceleration_scaling != 1.0) {
            for (size_t i = 0; i < trajectory_out.accelerations.size(); ++i) {
                for (size_t j = 0; j < trajectory_out.accelerations[i].size(); ++j) {
                    trajectory_out.accelerations[i][j] *= max_acceleration_scaling;
                }
            }
        }
        pimpl_->setLastError(PlannerStatus::SUCCESS, "");
        return true;
    }

    pimpl_->setLastError(PlannerStatus::TRAJECTORY_FAILED, "Native circular trajectory optimization failed.");
    return false;
}

} // namespace robot_planner
