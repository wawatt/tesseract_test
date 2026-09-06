#include "planner_impl.h"

namespace robot_planner {

bool RobotPlanner::checkCollisionDetailed(const std::vector<double>& joint_angles, 
                                          std::vector<ContactInfo>& contacts_out, 
                                          double contact_distance) {
    contacts_out.clear();
    pimpl_->last_known_joints_ = joint_angles;
    if (pimpl_->vampReady() && joint_angles.size() == 6) {
        bool hit = pimpl_->vamp_loader_->checkCollisionDetailed(joint_angles, contacts_out, contact_distance);
        if (hit && !contacts_out.empty()) {
            pimpl_->setLastError(PlannerStatus::COLLISION_DETECTED,
                "Contact detected between '" + contacts_out[0].link_name1 + "' and '" + contacts_out[0].link_name2 +
                "' (distance: " + std::to_string(contacts_out[0].distance) + "m)");
            return true;
        }
        pimpl_->setLastError(PlannerStatus::SUCCESS, "");
        return false;
    }
    if (!pimpl_->env_) {
        pimpl_->setLastError(PlannerStatus::NOT_INITIALIZED, "Environment not initialized.");
        return false;
    }
    auto active_link_names = pimpl_->env_->getActiveLinkNames();
    std::vector<std::string> joint_names = pimpl_->env_->getJointGroup(pimpl_->manipulator_name_)->getJointNames();
    if (joint_angles.size() != joint_names.size()) {
        pimpl_->setLastError(PlannerStatus::INVALID_ARGUMENTS, "joint_angles size mismatch.");
        return false;
    }
    pimpl_->last_known_joints_ = joint_angles;
    Eigen::VectorXd joints = Eigen::Map<const Eigen::VectorXd>(joint_angles.data(), joint_angles.size());
    tesseract::scene_graph::SceneState state = pimpl_->env_->getState(joint_names, joints);
    
    tesseract::collision::DiscreteContactManager::Ptr manager = pimpl_->env_->getDiscreteContactManager();
    manager->setActiveCollisionObjects(active_link_names);
    manager->setCollisionObjectsTransform(state.link_transforms);

    tesseract::collision::ContactRequest req(tesseract::collision::ContactTestType::ALL);
    req.calculate_penetration = true;
    req.calculate_distance = true;
    if (contact_distance > 0.0) {
        manager->setDefaultCollisionMargin(contact_distance);
    }

    tesseract::collision::ContactResultMap contact_results;
    manager->contactTest(contact_results, req);

    bool in_contact = false;
    for (const auto& pair : contact_results) {
        for (const auto& r : pair.second) {
            if (contact_distance <= 0.0 && r.distance > 0.0) continue;
            ContactInfo info;
            info.link_name1 = r.link_names[0];
            info.link_name2 = r.link_names[1];
            info.distance = r.distance;
            info.point1 = { r.nearest_points[0].x(), r.nearest_points[0].y(), r.nearest_points[0].z() };
            info.point2 = { r.nearest_points[1].x(), r.nearest_points[1].y(), r.nearest_points[1].z() };
            info.normal = { r.normal.x(), r.normal.y(), r.normal.z() };
            contacts_out.push_back(info);
            in_contact = true;
        }
    }
    if (in_contact && !contacts_out.empty()) {
        pimpl_->setLastError(PlannerStatus::COLLISION_DETECTED, 
            "Contact detected between '" + contacts_out[0].link_name1 + "' and '" + contacts_out[0].link_name2 + 
            "' (distance: " + std::to_string(contacts_out[0].distance) + "m)");
    } else {
        pimpl_->setLastError(PlannerStatus::SUCCESS, "");
    }
    return in_contact;
}

bool RobotPlanner::checkCollision(const std::vector<double>& joint_angles) {
    pimpl_->last_known_joints_ = joint_angles;
    if (!pimpl_->vampReady() || joint_angles.size() != 6) {
        pimpl_->setLastError(PlannerStatus::NOT_INITIALIZED, "VAMP backend is not loaded or joint vector is not 6-DOF.");
        return false;
    }
    bool col = pimpl_->vamp_loader_->checkCollision(joint_angles);
    if (col) {
        pimpl_->setLastError(PlannerStatus::COLLISION_DETECTED, "Collision detected by VAMP SIMD engine.");
        return true;
    }
    pimpl_->setLastError(PlannerStatus::SUCCESS, "");
    return false;
}

bool RobotPlanner::validateTrajectory(const JointTrajectory& trajectory, 
                                      int* failed_waypoint_index, 
                                      std::string* reason) {
    if (trajectory.empty()) {
        pimpl_->setLastError(PlannerStatus::INVALID_ARGUMENTS, "Trajectory is empty.");
        if (failed_waypoint_index) *failed_waypoint_index = -1;
        if (reason) *reason = "Trajectory is empty.";
        return false;
    }

    size_t num_points = trajectory.size();
    size_t num_joints = pimpl_->joint_limits_min_.size();

    for (size_t i = 0; i < num_points; ++i) {
        const auto& pos = trajectory.positions[i];
        if (pos.size() != num_joints) {
            std::string msg = "Waypoint " + std::to_string(i) + ": position dimension mismatch (expected " + 
                              std::to_string(num_joints) + ", got " + std::to_string(pos.size()) + ").";
            pimpl_->setLastError(PlannerStatus::INVALID_ARGUMENTS, msg);
            if (failed_waypoint_index) *failed_waypoint_index = static_cast<int>(i);
            if (reason) *reason = msg;
            return false;
        }

        // 1. Joint limit check
        for (size_t j = 0; j < num_joints; ++j) {
            if (pos[j] < pimpl_->joint_limits_min_[j] - 1e-4 || pos[j] > pimpl_->joint_limits_max_[j] + 1e-4) {
                std::string msg = "Waypoint " + std::to_string(i) + ": Joint " + std::to_string(j + 1) + 
                                  " position (" + std::to_string(pos[j]) + " rad) exceeds physical limits [" + 
                                  std::to_string(pimpl_->joint_limits_min_[j]) + ", " + 
                                  std::to_string(pimpl_->joint_limits_max_[j]) + "].";
                pimpl_->setLastError(PlannerStatus::JOINT_LIMIT_VIOLATED, msg);
                if (failed_waypoint_index) *failed_waypoint_index = static_cast<int>(i);
                if (reason) *reason = msg;
                return false;
            }
        }

        // 2. Velocity limit check
        if (i < trajectory.velocities.size() && !trajectory.velocities[i].empty()) {
            const auto& vel = trajectory.velocities[i];
            for (size_t j = 0; j < std::min(num_joints, vel.size()); ++j) {
                if (j < pimpl_->joint_vel_limits_.size() && pimpl_->joint_vel_limits_[j] > 0.0) {
                    if (std::abs(vel[j]) > pimpl_->joint_vel_limits_[j] * 1.001) {
                        std::string msg = "Waypoint " + std::to_string(i) + ": Joint " + std::to_string(j + 1) + 
                                          " velocity (" + std::to_string(vel[j]) + " rad/s) exceeds max limit " + 
                                          std::to_string(pimpl_->joint_vel_limits_[j]) + " rad/s.";
                        pimpl_->setLastError(PlannerStatus::TRAJECTORY_FAILED, msg);
                        if (failed_waypoint_index) *failed_waypoint_index = static_cast<int>(i);
                        if (reason) *reason = msg;
                        return false;
                    }
                }
            }
        }

        // 3. Acceleration limit check
        if (i < trajectory.accelerations.size() && !trajectory.accelerations[i].empty()) {
            const auto& acc = trajectory.accelerations[i];
            for (size_t j = 0; j < std::min(num_joints, acc.size()); ++j) {
                if (j < pimpl_->joint_acc_limits_.size() && pimpl_->joint_acc_limits_[j] > 0.0) {
                    if (std::abs(acc[j]) > pimpl_->joint_acc_limits_[j] * 1.001) {
                        std::string msg = "Waypoint " + std::to_string(i) + ": Joint " + std::to_string(j + 1) + 
                                          " acceleration (" + std::to_string(acc[j]) + " rad/s^2) exceeds max limit " + 
                                          std::to_string(pimpl_->joint_acc_limits_[j]) + " rad/s^2.";
                        pimpl_->setLastError(PlannerStatus::TRAJECTORY_FAILED, msg);
                        if (failed_waypoint_index) *failed_waypoint_index = static_cast<int>(i);
                        if (reason) *reason = msg;
                        return false;
                    }
                }
            }
        }

        // 4. Monotonic timestamp check
        if (i < trajectory.time_stamps.size()) {
            if (i > 0 && trajectory.time_stamps[i] <= trajectory.time_stamps[i - 1]) {
                std::string msg = "Waypoint " + std::to_string(i) + ": Timestamp (" + 
                                  std::to_string(trajectory.time_stamps[i]) + "s) is not strictly increasing from previous (" + 
                                  std::to_string(trajectory.time_stamps[i - 1]) + "s).";
                pimpl_->setLastError(PlannerStatus::TRAJECTORY_FAILED, msg);
                if (failed_waypoint_index) *failed_waypoint_index = static_cast<int>(i);
                if (reason) *reason = msg;
                return false;
            }
        }

        // 5. Collision check (VAMP AVX2 SIMD)
        if (checkCollision(pos)) {
            std::string msg = "Waypoint " + std::to_string(i) + ": " + pimpl_->last_error_;
            pimpl_->setLastError(PlannerStatus::COLLISION_DETECTED, msg);
            if (failed_waypoint_index) *failed_waypoint_index = static_cast<int>(i);
            if (reason) *reason = msg;
            return false;
        }
    }

    pimpl_->setLastError(PlannerStatus::SUCCESS, "");
    if (failed_waypoint_index) *failed_waypoint_index = -1;
    if (reason) *reason = "Trajectory validation passed.";
    return true;
}

} // namespace robot_planner
