#include "planner_impl.h"

namespace robot_planner {

static int mapLinkNameToIndex(const std::string& link_name) {
    if (link_name.find("tool") != std::string::npos || link_name.find("flange") != std::string::npos ||
        link_name.find("link_6") != std::string::npos || link_name.find("J6") != std::string::npos) return 6;
    if (link_name.find("link_5") != std::string::npos || link_name.find("J5") != std::string::npos) return 5;
    if (link_name.find("link_4") != std::string::npos || link_name.find("J4") != std::string::npos) return 4;
    if (link_name.find("link_3") != std::string::npos || link_name.find("J3") != std::string::npos) return 3;
    if (link_name.find("link_2") != std::string::npos || link_name.find("J2") != std::string::npos) return 2;
    if (link_name.find("link_1") != std::string::npos || link_name.find("J1") != std::string::npos) return 1;
    return 6; // default to end effector link
}

static std::vector<double> fitSpheresToObject(const ObstacleGeometryInfo& geom, const Eigen::Isometry3d& rel_tf) {
    std::vector<double> spheres; // flat x, y, z, r

    if (geom.type == ObstacleShapeType::BOX) {
        double dx = geom.dim_x;
        double dy = geom.dim_y;
        double dz = geom.dim_z;

        double L = dx;
        double W = dy;
        double H = dz;
        int long_axis = 0; // 0=X, 1=Y, 2=Z
        if (dy > L) { L = dy; W = dx; H = dz; long_axis = 1; }
        if (dz > L) { L = dz; W = dx; H = dy; long_axis = 2; }

        double max_cross = std::max(W, H);
        if (max_cross < 1e-4) max_cross = 0.05;

        int k = std::max(1, static_cast<int>(std::ceil(L / max_cross)));
        k = std::min(16, k);

        double delta = L / k;
        double radius = 0.5 * std::sqrt(W * W + H * H + delta * delta);

        for (int i = 0; i < k; ++i) {
            double offset = -0.5 * L + (i + 0.5) * delta;
            Eigen::Vector3d pt_box(0, 0, 0);
            if (long_axis == 0) pt_box.x() = offset;
            else if (long_axis == 1) pt_box.y() = offset;
            else pt_box.z() = offset;

            Eigen::Vector3d pt_link = rel_tf * pt_box;
            spheres.push_back(pt_link.x());
            spheres.push_back(pt_link.y());
            spheres.push_back(pt_link.z());
            spheres.push_back(radius);
        }
    } else if (geom.type == ObstacleShapeType::SPHERE) {
        Eigen::Vector3d pt_link = rel_tf * Eigen::Vector3d(0, 0, 0);
        spheres.push_back(pt_link.x());
        spheres.push_back(pt_link.y());
        spheres.push_back(pt_link.z());
        spheres.push_back(geom.radius);
    } else if (geom.type == ObstacleShapeType::CYLINDER || geom.type == ObstacleShapeType::CAPSULE) {
        double R = geom.radius;
        double L = geom.length;
        int k = std::max(1, static_cast<int>(std::ceil(L / (2.0 * R))));
        k = std::min(16, k);
        double delta = L / k;
        double radius = std::sqrt(R * R + 0.25 * delta * delta);

        for (int i = 0; i < k; ++i) {
            double z_off = -0.5 * L + (i + 0.5) * delta;
            Eigen::Vector3d pt_cyl(0, 0, z_off);
            Eigen::Vector3d pt_link = rel_tf * pt_cyl;
            spheres.push_back(pt_link.x());
            spheres.push_back(pt_link.y());
            spheres.push_back(pt_link.z());
            spheres.push_back(radius);
        }
    } else {
        if (!geom.aabb_array.empty()) {
            for (size_t i = 0; i + 5 < geom.aabb_array.size(); i += 6) {
                double min_x = geom.aabb_array[i];
                double min_y = geom.aabb_array[i+1];
                double min_z = geom.aabb_array[i+2];
                double max_x = geom.aabb_array[i+3];
                double max_y = geom.aabb_array[i+4];
                double max_z = geom.aabb_array[i+5];

                Eigen::Vector3d center(0.5 * (min_x + max_x), 0.5 * (min_y + max_y), 0.5 * (min_z + max_z));
                double dx = max_x - min_x;
                double dy = max_y - min_y;
                double dz = max_z - min_z;
                double radius = 0.5 * std::sqrt(dx*dx + dy*dy + dz*dz);

                Eigen::Vector3d pt_link = rel_tf * center;
                spheres.push_back(pt_link.x());
                spheres.push_back(pt_link.y());
                spheres.push_back(pt_link.z());
                spheres.push_back(radius);
            }
        } else {
            Eigen::Vector3d pt_link = rel_tf * Eigen::Vector3d(0, 0, 0);
            spheres.push_back(pt_link.x());
            spheres.push_back(pt_link.y());
            spheres.push_back(pt_link.z());
            spheres.push_back(0.2);
        }
    }
    return spheres;
}

bool RobotPlanner::addBox(const std::string& name, double x, double y, double z, double dim_x, double dim_y, double dim_z) {
    if (pimpl_->vamp_loader_ && pimpl_->vamp_loader_->isLoaded()) {
        pimpl_->vamp_loader_->addBox(name, x, y, z, dim_x, dim_y, dim_z);
    }
    pimpl_->obstacle_names_.insert(name);

    ObstacleGeometryInfo geom;
    geom.name = name;
    geom.type = ObstacleShapeType::BOX;
    geom.initial_pose = Eigen::Isometry3d::Identity();
    geom.initial_pose.translation() = Eigen::Vector3d(x, y, z);
    geom.dim_x = dim_x;
    geom.dim_y = dim_y;
    geom.dim_z = dim_z;
    pimpl_->obstacle_geometries_[name] = geom;

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

    ObstacleGeometryInfo geom;
    geom.name = name;
    geom.type = ObstacleShapeType::SPHERE;
    geom.initial_pose = Eigen::Isometry3d::Identity();
    geom.initial_pose.translation() = Eigen::Vector3d(x, y, z);
    geom.radius = radius;
    pimpl_->obstacle_geometries_[name] = geom;

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

    ObstacleGeometryInfo geom_cyl;
    geom_cyl.name = name;
    geom_cyl.type = ObstacleShapeType::CYLINDER;
    geom_cyl.initial_pose = tf;
    geom_cyl.radius = radius;
    geom_cyl.length = length;
    pimpl_->obstacle_geometries_[name] = geom_cyl;

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

    ObstacleGeometryInfo geom_cap;
    geom_cap.name = name;
    geom_cap.type = ObstacleShapeType::CAPSULE;
    geom_cap.initial_pose = tf;
    geom_cap.radius = radius;
    geom_cap.length = length;
    pimpl_->obstacle_geometries_[name] = geom_cap;

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

    std::vector<double> aabbs;
    if (pimpl_->vamp_loader_ && pimpl_->vamp_loader_->isLoaded()) {
        aabbs = convertMeshToAABBs(vertices, faces, pose, 0.04);
        pimpl_->vamp_loader_->addBoxes(name, aabbs);
    }
    pimpl_->obstacle_names_.insert(name);

    ObstacleGeometryInfo geom_mesh;
    geom_mesh.name = name;
    geom_mesh.type = ObstacleShapeType::MESH;
    geom_mesh.initial_pose = Eigen::Isometry3d::Identity();
    geom_mesh.initial_pose.translation() = Eigen::Vector3d(pose[0], pose[1], pose[2]);
    Eigen::Quaterniond q_mesh(pose[6], pose[3], pose[4], pose[5]);
    geom_mesh.initial_pose.linear() = q_mesh.matrix();
    geom_mesh.aabb_array = aabbs;
    pimpl_->obstacle_geometries_[name] = geom_mesh;

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
    std::vector<double> aabbs;
    if (pimpl_->vamp_loader_ && pimpl_->vamp_loader_->isLoaded()) {
        aabbs = convertPointCloudToAABBs(points, resolution, pose);
        pimpl_->vamp_loader_->addBoxes(name, aabbs);
    }
    pimpl_->obstacle_names_.insert(name);

    ObstacleGeometryInfo geom_pc;
    geom_pc.name = name;
    geom_pc.type = ObstacleShapeType::POINT_CLOUD;
    geom_pc.initial_pose = Eigen::Isometry3d::Identity();
    geom_pc.initial_pose.translation() = Eigen::Vector3d(pose[0], pose[1], pose[2]);
    Eigen::Quaterniond q_pc(pose[6], pose[3], pose[4], pose[5]);
    geom_pc.initial_pose.linear() = q_pc.matrix();
    geom_pc.aabb_array = aabbs;
    pimpl_->obstacle_geometries_[name] = geom_pc;

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
        pimpl_->vamp_loader_->removeAttachedSpheres(name);
    }
    pimpl_->obstacle_names_.erase(name);
    pimpl_->attached_obstacles_.erase(name);
    pimpl_->obstacle_geometries_.erase(name);

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
    if (pimpl_->vamp_loader_ && pimpl_->vamp_loader_->isLoaded()) {
        pimpl_->vamp_loader_->clearAttachedSpheres();
    }
    pimpl_->obstacle_names_.clear();
    pimpl_->attached_obstacles_.clear();
    pimpl_->obstacle_geometries_.clear();
    pimpl_->setLastError(all_ok ? PlannerStatus::SUCCESS : PlannerStatus::INTERNAL_ERROR, all_ok ? "" : "Failed to clear all obstacles.");
    return all_ok;
}

std::vector<std::string> RobotPlanner::getObstacleNames() const {
    return std::vector<std::string>(pimpl_->obstacle_names_.begin(), pimpl_->obstacle_names_.end());
}

bool RobotPlanner::hasObstacle(const std::string& name) const {
    return pimpl_->obstacle_names_.find(name) != pimpl_->obstacle_names_.end();
}

bool RobotPlanner::attachObject(const std::string& obstacle_name, const std::string& target_link, const std::vector<double>& current_joints) {
    if (!hasObstacle(obstacle_name)) {
        pimpl_->setLastError(PlannerStatus::OBSTACLE_NOT_FOUND, "Obstacle '" + obstacle_name + "' not found.");
        return false;
    }
    if (!pimpl_->env_) return false;

    std::string actual_link = target_link.empty() ? pimpl_->tool_link_ : target_link;

    const std::vector<double>& eff_q = !current_joints.empty() ? current_joints : pimpl_->last_known_joints_;
    tesseract::scene_graph::SceneState current_state;
    if (!eff_q.empty()) {
        std::vector<std::string> joint_names = pimpl_->env_->getJointGroup(pimpl_->manipulator_name_)->getJointNames();
        if (eff_q.size() == joint_names.size()) {
            Eigen::VectorXd j = Eigen::Map<const Eigen::VectorXd>(eff_q.data(), eff_q.size());
            current_state = pimpl_->env_->getState(joint_names, j);
        } else {
            current_state = pimpl_->env_->getState();
        }
    } else {
        current_state = pimpl_->env_->getState();
    }

    auto it_obs = current_state.link_transforms.find(obstacle_name);
    auto it_target = current_state.link_transforms.find(actual_link);

    if (it_obs == current_state.link_transforms.end() || it_target == current_state.link_transforms.end()) {
        pimpl_->setLastError(PlannerStatus::INTERNAL_ERROR, "Failed to query link transforms for attaching.");
        return false;
    }

    Eigen::Isometry3d obs_tf = it_obs->second;
    Eigen::Isometry3d target_tf = it_target->second;
    Eigen::Isometry3d rel_tf = target_tf.inverse() * obs_tf;

    tesseract::environment::Commands cmds;
    tesseract::scene_graph::Joint joint(obstacle_name + "_joint");
    joint.parent_link_name = actual_link;
    joint.child_link_name = obstacle_name;
    joint.type = tesseract::scene_graph::JointType::FIXED;
    joint.parent_to_joint_origin_transform = rel_tf;
    cmds.push_back(std::make_shared<tesseract::environment::MoveLinkCommand>(joint));

    // 挂载后允许物体与挂载工具连杆免检，防止自我碰撞误报
    tesseract::common::AllowedCollisionMatrix acm;
    acm.addAllowedCollision(obstacle_name, actual_link, "Attached");
    cmds.push_back(std::make_shared<tesseract::environment::ModifyAllowedCollisionsCommand>(
        acm, tesseract::environment::ModifyAllowedCollisionsType::ADD));

    if (!pimpl_->env_->applyCommands(cmds)) {
        pimpl_->setLastError(PlannerStatus::INTERNAL_ERROR, "Failed to apply MoveLinkCommand in Tesseract.");
        return false;
    }

    pimpl_->attached_obstacles_[obstacle_name] = actual_link;

    // VAMP SIMD Collision Kernel Attachment Synchronization (cuRobo AttachmentManager)
    if (pimpl_->vamp_loader_ && pimpl_->vamp_loader_->isLoaded()) {
        auto it_geom = pimpl_->obstacle_geometries_.find(obstacle_name);
        if (it_geom != pimpl_->obstacle_geometries_.end()) {
            int link_idx = mapLinkNameToIndex(actual_link);
            std::string vamp_link_name = (link_idx >= 6) ? "J6_link" : ("J" + std::to_string(link_idx) + "_link");
            if (link_idx == 0) vamp_link_name = "base_link";

            auto it_vamp_link = current_state.link_transforms.find(vamp_link_name);
            Eigen::Isometry3d vamp_link_tf = (it_vamp_link != current_state.link_transforms.end()) ? it_vamp_link->second : target_tf;
            Eigen::Isometry3d rel_tf_vamp = vamp_link_tf.inverse() * obs_tf;

            std::vector<double> fitted_spheres = fitSpheresToObject(it_geom->second, rel_tf_vamp);
            if (!fitted_spheres.empty()) {
                pimpl_->vamp_loader_->addAttachedSpheres(obstacle_name, link_idx, fitted_spheres.data(), static_cast<int>(fitted_spheres.size() / 4));
            }
        }
        // CRUCIAL: Remove the static obstacle from the world collision space so the robot doesn't collide with its own ghost
        pimpl_->vamp_loader_->removeObstacle(obstacle_name);
    }

    pimpl_->setLastError(PlannerStatus::SUCCESS, "");
    return true;
}

bool RobotPlanner::detachObject(const std::string& obstacle_name, const std::vector<double>& current_joints) {
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

    const std::vector<double>& eff_q = !current_joints.empty() ? current_joints : pimpl_->last_known_joints_;
    tesseract::scene_graph::SceneState current_state;
    if (!eff_q.empty()) {
        std::vector<std::string> joint_names = pimpl_->env_->getJointGroup(pimpl_->manipulator_name_)->getJointNames();
        if (eff_q.size() == joint_names.size()) {
            Eigen::VectorXd j = Eigen::Map<const Eigen::VectorXd>(eff_q.data(), eff_q.size());
            current_state = pimpl_->env_->getState(joint_names, j);
        } else {
            current_state = pimpl_->env_->getState();
        }
    } else {
        current_state = pimpl_->env_->getState();
    }

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

    // VAMP SIMD Collision Kernel Detachment Synchronization
    if (pimpl_->vamp_loader_ && pimpl_->vamp_loader_->isLoaded()) {
        pimpl_->vamp_loader_->removeAttachedSpheres(obstacle_name);

        // Re-enable as static world obstacle at its newly placed world pose
        auto it_geom = pimpl_->obstacle_geometries_.find(obstacle_name);
        if (it_geom != pimpl_->obstacle_geometries_.end()) {
            it_geom->second.initial_pose = world_tf;
            const auto& geom = it_geom->second;
            if (geom.type == ObstacleShapeType::BOX) {
                Eigen::Matrix3d R = world_tf.rotation();
                Eigen::Vector3d half_ext(geom.dim_x * 0.5, geom.dim_y * 0.5, geom.dim_z * 0.5);
                Eigen::Vector3d aabb_half = R.cwiseAbs() * half_ext;
                pimpl_->vamp_loader_->addBox(obstacle_name,
                                             world_tf.translation().x(), world_tf.translation().y(), world_tf.translation().z(),
                                             2.0 * aabb_half.x(), 2.0 * aabb_half.y(), 2.0 * aabb_half.z());
            } else if (geom.type == ObstacleShapeType::SPHERE) {
                double d = 2.0 * geom.radius;
                pimpl_->vamp_loader_->addBox(obstacle_name,
                                             world_tf.translation().x(), world_tf.translation().y(), world_tf.translation().z(),
                                             d, d, d);
            } else if (geom.type == ObstacleShapeType::CYLINDER || geom.type == ObstacleShapeType::CAPSULE) {
                Eigen::Matrix3d R = world_tf.rotation();
                double total_len = (geom.type == ObstacleShapeType::CAPSULE) ? (geom.length * 0.5 + geom.radius) : (geom.length * 0.5);
                Eigen::Vector3d local_half(geom.radius, geom.radius, total_len);
                Eigen::Vector3d world_half = R.cwiseAbs() * local_half;
                pimpl_->vamp_loader_->addBox(obstacle_name,
                                             world_tf.translation().x(), world_tf.translation().y(), world_tf.translation().z(),
                                             2.0 * world_half.x(), 2.0 * world_half.y(), 2.0 * world_half.z());
            } else if (!geom.aabb_array.empty()) {
                pimpl_->vamp_loader_->addBoxes(obstacle_name, geom.aabb_array);
            }
        }
    }

    pimpl_->setLastError(PlannerStatus::SUCCESS, "");
    return true;
}

} // namespace robot_planner
