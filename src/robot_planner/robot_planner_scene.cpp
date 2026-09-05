#include "planner_impl.h"

namespace robot_planner {

bool RobotPlanner::addBox(const std::string& name, double x, double y, double z, double dim_x, double dim_y, double dim_z) {
    if (pimpl_->vamp_loader_ && pimpl_->vamp_loader_->isLoaded()) {
        pimpl_->vamp_loader_->addBox(name, x, y, z, dim_x, dim_y, dim_z);
    }
    pimpl_->obstacle_names_.insert(name);

    if (!pimpl_->env_) return false;
    
    tesseract::scene_graph::Link link(name);
    
    tesseract::scene_graph::Visual::Ptr visual = std::make_shared<tesseract::scene_graph::Visual>();
    visual->origin = Eigen::Isometry3d::Identity();
    visual->geometry = std::make_shared<tesseract::geometry::Box>(dim_x, dim_y, dim_z);
    link.visual.push_back(visual);
    
    tesseract::scene_graph::Collision::Ptr collision = std::make_shared<tesseract::scene_graph::Collision>();
    collision->origin = visual->origin;
    collision->geometry = visual->geometry;
    link.collision.push_back(collision);
    
    tesseract::scene_graph::Joint joint(name + "_joint");
    joint.parent_link_name = pimpl_->env_->getRootLinkName();
    joint.child_link_name = name;
    joint.type = tesseract::scene_graph::JointType::FIXED;
    joint.parent_to_joint_origin_transform = Eigen::Isometry3d::Identity();
    joint.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(x, y, z);
    
    auto cmd = std::make_shared<tesseract::environment::AddLinkCommand>(link, joint);
    bool ok = pimpl_->env_->applyCommand(cmd);
    pimpl_->setLastError(ok ? PlannerStatus::SUCCESS : PlannerStatus::INTERNAL_ERROR, ok ? "" : "Failed to add Box link.");
    return ok;
}

bool RobotPlanner::addSphere(const std::string& name, double x, double y, double z, double radius) {
    if (radius <= 0.0) {
        pimpl_->setLastError(PlannerStatus::INVALID_ARGUMENTS, "Sphere radius must be positive.");
        return false;
    }
    if (pimpl_->vamp_loader_ && pimpl_->vamp_loader_->isLoaded()) {
        double d = 2.0 * radius;
        pimpl_->vamp_loader_->addBox(name, x, y, z, d, d, d);
    }
    pimpl_->obstacle_names_.insert(name);

    if (!pimpl_->env_) return false;

    tesseract::scene_graph::Link link(name);
    auto visual = std::make_shared<tesseract::scene_graph::Visual>();
    visual->origin = Eigen::Isometry3d::Identity();
    visual->geometry = std::make_shared<tesseract::geometry::Sphere>(radius);
    link.visual.push_back(visual);

    auto collision = std::make_shared<tesseract::scene_graph::Collision>();
    collision->origin = visual->origin;
    collision->geometry = visual->geometry;
    link.collision.push_back(collision);

    tesseract::scene_graph::Joint joint(name + "_joint");
    joint.parent_link_name = pimpl_->env_->getRootLinkName();
    joint.child_link_name = name;
    joint.type = tesseract::scene_graph::JointType::FIXED;
    joint.parent_to_joint_origin_transform = Eigen::Isometry3d::Identity();
    joint.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(x, y, z);

    auto cmd = std::make_shared<tesseract::environment::AddLinkCommand>(link, joint);
    bool ok = pimpl_->env_->applyCommand(cmd);
    pimpl_->setLastError(ok ? PlannerStatus::SUCCESS : PlannerStatus::INTERNAL_ERROR, ok ? "" : "Failed to add Sphere link.");
    return ok;
}

bool RobotPlanner::addCylinder(const std::string& name, double radius, double length, const std::vector<double>& pose) {
    if (radius <= 0.0 || length <= 0.0) {
        pimpl_->setLastError(PlannerStatus::INVALID_ARGUMENTS, "Cylinder radius and length must be positive.");
        return false;
    }
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    if (pose.size() >= 7) {
        tf.translation() = Eigen::Vector3d(pose[0], pose[1], pose[2]);
        Eigen::Quaterniond q(pose[6], pose[3], pose[4], pose[5]);
        tf.linear() = q.normalized().toRotationMatrix();
    }

    if (pimpl_->vamp_loader_ && pimpl_->vamp_loader_->isLoaded()) {
        Eigen::Matrix3d R = tf.rotation();
        Eigen::Vector3d local_half(radius, radius, length * 0.5);
        Eigen::Vector3d world_half = R.cwiseAbs() * local_half;
        pimpl_->vamp_loader_->addBox(name, tf.translation().x(), tf.translation().y(), tf.translation().z(),
                                     2.0 * world_half.x(), 2.0 * world_half.y(), 2.0 * world_half.z());
    }
    pimpl_->obstacle_names_.insert(name);

    if (!pimpl_->env_) return false;

    tesseract::scene_graph::Link link(name);
    auto visual = std::make_shared<tesseract::scene_graph::Visual>();
    visual->origin = Eigen::Isometry3d::Identity();
    visual->geometry = std::make_shared<tesseract::geometry::Cylinder>(radius, length);
    link.visual.push_back(visual);

    auto collision = std::make_shared<tesseract::scene_graph::Collision>();
    collision->origin = visual->origin;
    collision->geometry = visual->geometry;
    link.collision.push_back(collision);

    tesseract::scene_graph::Joint joint(name + "_joint");
    joint.parent_link_name = pimpl_->env_->getRootLinkName();
    joint.child_link_name = name;
    joint.type = tesseract::scene_graph::JointType::FIXED;
    joint.parent_to_joint_origin_transform = tf;

    auto cmd = std::make_shared<tesseract::environment::AddLinkCommand>(link, joint);
    bool ok = pimpl_->env_->applyCommand(cmd);
    pimpl_->setLastError(ok ? PlannerStatus::SUCCESS : PlannerStatus::INTERNAL_ERROR, ok ? "" : "Failed to add Cylinder link.");
    return ok;
}

bool RobotPlanner::addCapsule(const std::string& name, double radius, double length, const std::vector<double>& pose) {
    if (radius <= 0.0 || length <= 0.0) {
        pimpl_->setLastError(PlannerStatus::INVALID_ARGUMENTS, "Capsule radius and length must be positive.");
        return false;
    }
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    if (pose.size() >= 7) {
        tf.translation() = Eigen::Vector3d(pose[0], pose[1], pose[2]);
        Eigen::Quaterniond q(pose[6], pose[3], pose[4], pose[5]);
        tf.linear() = q.normalized().toRotationMatrix();
    }

    if (pimpl_->vamp_loader_ && pimpl_->vamp_loader_->isLoaded()) {
        Eigen::Matrix3d R = tf.rotation();
        double total_half_len = length * 0.5 + radius;
        Eigen::Vector3d local_half(radius, radius, total_half_len);
        Eigen::Vector3d world_half = R.cwiseAbs() * local_half;
        pimpl_->vamp_loader_->addBox(name, tf.translation().x(), tf.translation().y(), tf.translation().z(),
                                     2.0 * world_half.x(), 2.0 * world_half.y(), 2.0 * world_half.z());
    }
    pimpl_->obstacle_names_.insert(name);

    if (!pimpl_->env_) return false;

    tesseract::scene_graph::Link link(name);
    auto visual = std::make_shared<tesseract::scene_graph::Visual>();
    visual->origin = Eigen::Isometry3d::Identity();
    visual->geometry = std::make_shared<tesseract::geometry::Capsule>(radius, length);
    link.visual.push_back(visual);

    auto collision = std::make_shared<tesseract::scene_graph::Collision>();
    collision->origin = visual->origin;
    collision->geometry = visual->geometry;
    link.collision.push_back(collision);

    tesseract::scene_graph::Joint joint(name + "_joint");
    joint.parent_link_name = pimpl_->env_->getRootLinkName();
    joint.child_link_name = name;
    joint.type = tesseract::scene_graph::JointType::FIXED;
    joint.parent_to_joint_origin_transform = tf;

    auto cmd = std::make_shared<tesseract::environment::AddLinkCommand>(link, joint);
    bool ok = pimpl_->env_->applyCommand(cmd);
    pimpl_->setLastError(ok ? PlannerStatus::SUCCESS : PlannerStatus::INTERNAL_ERROR, ok ? "" : "Failed to add Capsule link.");
    return ok;
}

bool RobotPlanner::addMesh(const std::string& name,
                           const std::vector<double>& vertices,
                           const std::vector<int>& faces,
                           const std::vector<double>& pose) {
    if (pose.size() < 7) {
        pimpl_->setLastError(PlannerStatus::INVALID_ARGUMENTS, "Pose must have 7 elements [x, y, z, qx, qy, qz, qw].");
        return false;
    }
    if (vertices.empty() || faces.empty()) {
        pimpl_->setLastError(PlannerStatus::INVALID_ARGUMENTS, "Mesh vertices or faces cannot be empty.");
        return false;
    }

    if (pimpl_->vamp_loader_ && pimpl_->vamp_loader_->isLoaded()) {
        auto aabbs = convertMeshToAABBs(vertices, faces, pose, 0.04);
        pimpl_->vamp_loader_->addBoxes(name, aabbs);
    }
    pimpl_->obstacle_names_.insert(name);

    if (!pimpl_->env_) return false;

    auto mesh_vertices = std::make_shared<tesseract::common::VectorVector3d>();
    for (size_t i = 0; i + 2 < vertices.size(); i += 3) {
        mesh_vertices->emplace_back(vertices[i], vertices[i+1], vertices[i+2]);
    }

    int num_triangles = faces.size() / 3;
    auto mesh_faces = std::make_shared<Eigen::VectorXi>(num_triangles * 4);
    for (int i = 0; i < num_triangles; ++i) {
        (*mesh_faces)[i * 4 + 0] = 3;
        (*mesh_faces)[i * 4 + 1] = faces[i * 3 + 0];
        (*mesh_faces)[i * 4 + 2] = faces[i * 3 + 1];
        (*mesh_faces)[i * 4 + 3] = faces[i * 3 + 2];
    }

    tesseract::scene_graph::Link link(name);
    
    tesseract::scene_graph::Visual::Ptr visual = std::make_shared<tesseract::scene_graph::Visual>();
    visual->origin = Eigen::Isometry3d::Identity();
    visual->geometry = std::make_shared<tesseract::geometry::Mesh>(mesh_vertices, mesh_faces);
    link.visual.push_back(visual);
    
    tesseract::scene_graph::Collision::Ptr collision = std::make_shared<tesseract::scene_graph::Collision>();
    collision->origin = visual->origin;
    collision->geometry = visual->geometry;
    link.collision.push_back(collision);
    
    tesseract::scene_graph::Joint joint(name + "_joint");
    joint.parent_link_name = pimpl_->env_->getRootLinkName();
    joint.child_link_name = name;
    joint.type = tesseract::scene_graph::JointType::FIXED;
    joint.parent_to_joint_origin_transform = Eigen::Isometry3d::Identity();
    joint.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(pose[0], pose[1], pose[2]);
    Eigen::Quaterniond q(pose[6], pose[3], pose[4], pose[5]);
    joint.parent_to_joint_origin_transform.linear() = q.matrix();
    
    auto cmd = std::make_shared<tesseract::environment::AddLinkCommand>(link, joint);
    bool ok = pimpl_->env_->applyCommand(cmd);
    pimpl_->setLastError(ok ? PlannerStatus::SUCCESS : PlannerStatus::INTERNAL_ERROR, ok ? "" : "Failed to add Mesh link.");
    return ok;
}

bool RobotPlanner::addPointCloud(const std::string& name,
                                 const std::vector<double>& points,
                                 double resolution,
                                 const std::vector<double>& pose) {
    if (pimpl_->vamp_loader_ && pimpl_->vamp_loader_->isLoaded()) {
        auto aabbs = convertPointCloudToAABBs(points, resolution, pose);
        pimpl_->vamp_loader_->addBoxes(name, aabbs);
    }
    pimpl_->obstacle_names_.insert(name);

    if (!pimpl_->env_) return false;
    
    tesseract::geometry::PointCloud point_cloud;
    for (size_t i = 0; i + 2 < points.size(); i += 3) {
        point_cloud.addPoint(points[i], points[i+1], points[i+2]);
    }

    auto octree = tesseract::geometry::createOctree(point_cloud, resolution, true, true);
    auto shared_octree = std::shared_ptr<const octomap::OcTree>(octree.release());
    
    tesseract::scene_graph::Link link(name);
    
    tesseract::scene_graph::Visual::Ptr visual = std::make_shared<tesseract::scene_graph::Visual>();
    visual->origin = Eigen::Isometry3d::Identity();
    visual->geometry = std::make_shared<tesseract::geometry::Octree>(shared_octree, tesseract::geometry::OctreeSubType::BOX);
    link.visual.push_back(visual);
    
    tesseract::scene_graph::Collision::Ptr collision = std::make_shared<tesseract::scene_graph::Collision>();
    collision->origin = visual->origin;
    collision->geometry = visual->geometry;
    link.collision.push_back(collision);
    
    tesseract::scene_graph::Joint joint(name + "_joint");
    joint.parent_link_name = pimpl_->env_->getRootLinkName();
    joint.child_link_name = name;
    joint.type = tesseract::scene_graph::JointType::FIXED;
    joint.parent_to_joint_origin_transform = Eigen::Isometry3d::Identity();
    joint.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(pose[0], pose[1], pose[2]);
    Eigen::Quaterniond q(pose[6], pose[3], pose[4], pose[5]);
    joint.parent_to_joint_origin_transform.linear() = q.matrix();
    
    auto cmd = std::make_shared<tesseract::environment::AddLinkCommand>(link, joint);
    bool ok = pimpl_->env_->applyCommand(cmd);
    pimpl_->setLastError(ok ? PlannerStatus::SUCCESS : PlannerStatus::INTERNAL_ERROR, ok ? "" : "Failed to add PointCloud link.");
    return ok;
}

bool RobotPlanner::setAllowedCollision(const std::string& link1, const std::string& link2, bool allowed) {
    if (!pimpl_->env_) {
        pimpl_->setLastError(PlannerStatus::NOT_INITIALIZED, "Environment not initialized.");
        return false;
    }
    tesseract::common::AllowedCollisionMatrix acm;
    auto cur_acm = pimpl_->env_->getAllowedCollisionMatrix();
    if (cur_acm) {
        acm = *cur_acm;
    }
    if (allowed) {
        acm.addAllowedCollision(link1, link2, "UserAllowed");
    } else {
        acm.removeAllowedCollision(link1, link2);
    }
    auto cmd = std::make_shared<tesseract::environment::ModifyAllowedCollisionsCommand>(
        acm, tesseract::environment::ModifyAllowedCollisionsType::REPLACE);
    bool ok = pimpl_->env_->applyCommand(cmd);
    pimpl_->setLastError(ok ? PlannerStatus::SUCCESS : PlannerStatus::INTERNAL_ERROR,
                         ok ? "" : "Failed to apply ModifyAllowedCollisionsCommand.");
    return ok;
}

bool RobotPlanner::isCollisionAllowed(const std::string& link1, const std::string& link2) const {
    if (!pimpl_->env_) return false;
    auto acm = pimpl_->env_->getAllowedCollisionMatrix();
    if (!acm) return false;
    return acm->isCollisionAllowed(link1, link2);
}

bool RobotPlanner::removeObstacle(const std::string& name) {
    if (pimpl_->vamp_loader_ && pimpl_->vamp_loader_->isLoaded()) {
        pimpl_->vamp_loader_->removeObstacle(name);
    }
    pimpl_->obstacle_names_.erase(name);
    pimpl_->attached_obstacles_.erase(name);

    if (!pimpl_->env_) return false;
    
    auto cmd = std::make_shared<tesseract::environment::RemoveLinkCommand>(name);
    bool ok = pimpl_->env_->applyCommand(cmd);
    pimpl_->setLastError(ok ? PlannerStatus::SUCCESS : PlannerStatus::OBSTACLE_NOT_FOUND, ok ? "" : "Failed to remove link.");
    return ok;
}

bool RobotPlanner::clearObstacles() {
    bool all_ok = true;
    std::vector<std::string> names(pimpl_->obstacle_names_.begin(), pimpl_->obstacle_names_.end());
    for (const auto& name : names) {
        if (!removeObstacle(name)) {
            all_ok = false;
        }
    }
    pimpl_->obstacle_names_.clear();
    pimpl_->attached_obstacles_.clear();
    pimpl_->setLastError(all_ok ? PlannerStatus::SUCCESS : PlannerStatus::INTERNAL_ERROR, all_ok ? "" : "Failed to clear all obstacles.");
    return all_ok;
}

std::vector<std::string> RobotPlanner::getObstacleNames() const {
    return std::vector<std::string>(pimpl_->obstacle_names_.begin(), pimpl_->obstacle_names_.end());
}

bool RobotPlanner::hasObstacle(const std::string& name) const {
    return pimpl_->obstacle_names_.find(name) != pimpl_->obstacle_names_.end();
}

bool RobotPlanner::attachObject(const std::string& obstacle_name, const std::string& target_link) {
    if (!hasObstacle(obstacle_name)) {
        pimpl_->setLastError(PlannerStatus::OBSTACLE_NOT_FOUND, "Obstacle '" + obstacle_name + "' not found.");
        return false;
    }
    if (!pimpl_->env_) return false;

    auto current_state = pimpl_->env_->getState();
    auto it_obs = current_state.link_transforms.find(obstacle_name);
    auto it_target = current_state.link_transforms.find(target_link);

    if (it_obs == current_state.link_transforms.end() || it_target == current_state.link_transforms.end()) {
        pimpl_->setLastError(PlannerStatus::INTERNAL_ERROR, "Failed to query link transforms for attaching.");
        return false;
    }

    Eigen::Isometry3d obs_tf = it_obs->second;
    Eigen::Isometry3d target_tf = it_target->second;
    Eigen::Isometry3d rel_tf = target_tf.inverse() * obs_tf;

    tesseract::environment::Commands cmds;
    tesseract::scene_graph::Joint joint(obstacle_name + "_joint");
    joint.parent_link_name = target_link;
    joint.child_link_name = obstacle_name;
    joint.type = tesseract::scene_graph::JointType::FIXED;
    joint.parent_to_joint_origin_transform = rel_tf;
    cmds.push_back(std::make_shared<tesseract::environment::MoveLinkCommand>(joint));

    // 挂载后允许物体与挂载工具连杆免检，防止自我碰撞误报
    tesseract::common::AllowedCollisionMatrix acm;
    acm.addAllowedCollision(obstacle_name, target_link, "Attached");
    cmds.push_back(std::make_shared<tesseract::environment::ModifyAllowedCollisionsCommand>(
        acm, tesseract::environment::ModifyAllowedCollisionsType::ADD));

    if (!pimpl_->env_->applyCommands(cmds)) {
        pimpl_->setLastError(PlannerStatus::INTERNAL_ERROR, "Failed to apply MoveLinkCommand in Tesseract.");
        return false;
    }

    pimpl_->attached_obstacles_[obstacle_name] = target_link;
    pimpl_->setLastError(PlannerStatus::SUCCESS, "");
    return true;
}

bool RobotPlanner::detachObject(const std::string& obstacle_name) {
    if (!hasObstacle(obstacle_name)) {
        pimpl_->setLastError(PlannerStatus::OBSTACLE_NOT_FOUND, "Obstacle '" + obstacle_name + "' not found.");
        return false;
    }
    if (!pimpl_->env_) return false;

    auto it_attached = pimpl_->attached_obstacles_.find(obstacle_name);
    if (it_attached == pimpl_->attached_obstacles_.end()) {
        return true; // Not currently attached
    }

    std::string attached_link = it_attached->second;
    auto current_state = pimpl_->env_->getState();
    auto it_obs = current_state.link_transforms.find(obstacle_name);
    if (it_obs == current_state.link_transforms.end()) {
        pimpl_->setLastError(PlannerStatus::INTERNAL_ERROR, "Failed to query link transform for detaching.");
        return false;
    }
    Eigen::Isometry3d world_tf = it_obs->second;

    tesseract::environment::Commands cmds;
    tesseract::scene_graph::Joint joint(obstacle_name + "_joint");
    joint.parent_link_name = pimpl_->env_->getRootLinkName();
    joint.child_link_name = obstacle_name;
    joint.type = tesseract::scene_graph::JointType::FIXED;
    joint.parent_to_joint_origin_transform = world_tf;
    cmds.push_back(std::make_shared<tesseract::environment::MoveLinkCommand>(joint));

    tesseract::common::AllowedCollisionMatrix acm;
    acm.addAllowedCollision(obstacle_name, attached_link, "Attached");
    cmds.push_back(std::make_shared<tesseract::environment::ModifyAllowedCollisionsCommand>(
        acm, tesseract::environment::ModifyAllowedCollisionsType::REMOVE));

    if (!pimpl_->env_->applyCommands(cmds)) {
        pimpl_->setLastError(PlannerStatus::INTERNAL_ERROR, "Failed to detach link in Tesseract.");
        return false;
    }

    pimpl_->attached_obstacles_.erase(it_attached);
    pimpl_->setLastError(PlannerStatus::SUCCESS, "");
    return true;
}

} // namespace robot_planner
