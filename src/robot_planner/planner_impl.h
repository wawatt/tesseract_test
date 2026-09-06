#pragma once

#include <robot_planner/robot_planner.h>

#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <unordered_set>
#include <unordered_map>
#include <cmath>
#include <chrono>
#include <algorithm>
#include <filesystem>

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/environment/environment.h>
#include <tesseract/environment/utils.h>
#include <tesseract/environment/commands/add_link_command.h>
#include <tesseract/environment/commands/remove_link_command.h>
#include <tesseract/environment/commands/move_link_command.h>
#include <tesseract/environment/commands/modify_allowed_collisions_command.h>
#include <tesseract/common/allowed_collision_matrix.h>
#include <tesseract/scene_graph/link.h>
#include <tesseract/scene_graph/joint.h>
#include <tesseract/scene_graph/graph.h>
#include <tesseract/common/resource_locator.h>

#include <tesseract/kinematics/forward_kinematics.h>
#include <tesseract/kinematics/inverse_kinematics.h>
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/kinematics/kinematic_group.h>

#include <tesseract/geometry/impl/mesh.h>
#include <tesseract/geometry/impl/box.h>
#include <tesseract/geometry/impl/sphere.h>
#include <tesseract/geometry/impl/cylinder.h>
#include <tesseract/geometry/impl/capsule.h>
#include <tesseract/geometry/impl/mesh_material.h>
#include <tesseract/geometry/impl/octree.h>
#include <tesseract/geometry/impl/octree_utils.h>
#include <tesseract/common/manipulator_info.h>
#include <tesseract/common/profile_dictionary.h>
#include <tesseract/scene_graph/scene_state.h>

#include <tesseract/collision/types.h>
#include <tesseract/collision/discrete_contact_manager.h>

// Planning headers
#include <tesseract/command_language/composite_instruction.h>
#include <tesseract/command_language/state_waypoint.h>
#include <tesseract/command_language/cartesian_waypoint.h>
#include <tesseract/command_language/joint_waypoint.h>
#include <tesseract/command_language/move_instruction.h>
#include <tesseract/command_language/utils.h>

#include <tesseract/task_composer/task_composer_context.h>
#include <tesseract/task_composer/task_composer_data_storage.h>
#include <tesseract/task_composer/task_composer_future.h>
#include <tesseract/task_composer/task_composer_executor.h>
#include <tesseract/task_composer/task_composer_node.h>
#include <tesseract/task_composer/task_composer_plugin_factory.h>

#include <tesseract/motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract/motion_planners/trajopt/profile/trajopt_default_move_profile.h>
#include <tesseract/motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_composite_profile.h>
#include <tesseract/motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_move_profile.h>
#include <tesseract/motion_planners/utils.h>
#include <tesseract/motion_planners/ompl/profile/ompl_real_vector_move_profile.h>
#include <tesseract/motion_planners/ompl/ompl_planner_configurator.h>

#include <robot_planner/vamp_dynamic_loader.h>
#include <robot_planner/opw_kinematics.h>

namespace robot_planner {

enum class ObstacleShapeType {
    BOX,
    SPHERE,
    CYLINDER,
    CAPSULE,
    MESH,
    POINT_CLOUD
};

struct ObstacleGeometryInfo {
    std::string name;
    ObstacleShapeType type;
    Eigen::Isometry3d initial_pose = Eigen::Isometry3d::Identity();
    double dim_x = 0.0;
    double dim_y = 0.0;
    double dim_z = 0.0;
    double radius = 0.0;
    double length = 0.0;
    std::vector<double> aabb_array;
};

struct RobotPlanner::Impl {
    std::shared_ptr<tesseract::environment::Environment> env_;
    std::string manipulator_name_;
    std::string base_link_;
    std::string tool_link_;
    
    // Task composer factory for planning
    std::unique_ptr<tesseract::task_composer::TaskComposerPluginFactory> factory_;

    // Optional dynamic loader for vamp_r2000ic.dll
    std::unique_ptr<VampDynamicLoader> vamp_loader_;
    PlannerBackend backend_ = PlannerBackend::VAMP;

    bool vampReady() const {
        return vamp_loader_ && vamp_loader_->isLoaded();
    }

    opw_kinematics::Parameters<double> opw_params_ = OpwKinematics::getFanucR2000iCParameters();
    std::vector<double> joint_origins_xyz_;
    std::vector<double> joint_origins_rpy_;
    std::vector<double> joint_axes_xyz_;

    // URDF Joint limits
    std::vector<double> joint_limits_min_;
    std::vector<double> joint_limits_max_;
    std::vector<double> joint_vel_limits_;
    std::vector<double> joint_acc_limits_;

    // Error diagnostics
    PlannerStatus last_status_ = PlannerStatus::SUCCESS;
    std::string last_error_;

    void setLastError(PlannerStatus status, const std::string& msg) {
        last_status_ = status;
        last_error_ = msg;
    }

    // Scene & Obstacle management
    std::unordered_set<std::string> obstacle_names_;
    std::unordered_map<std::string, std::string> attached_obstacles_; // obstacle_name -> link_name
    std::unordered_map<std::string, ObstacleGeometryInfo> obstacle_geometries_;
    std::vector<double> last_known_joints_;
};

// Flexible resource locator that searches relative to URDF path and CWD
class FlexibleResourceLocator : public tesseract::common::ResourceLocator {
public:
    FlexibleResourceLocator(std::filesystem::path base_dir)
        : base_dir_(std::move(base_dir)), fallback_(std::make_shared<tesseract::common::GeneralResourceLocator>()) {}

    std::shared_ptr<tesseract::common::Resource> locateResource(const std::string& url) const override {
        // 1. Try relative to URDF parent directory first
        std::filesystem::path p1 = base_dir_ / url;
        if (std::filesystem::exists(p1)) {
            return std::make_shared<tesseract::common::SimpleLocatedResource>(
                url, std::filesystem::absolute(p1).string(), std::make_shared<FlexibleResourceLocator>(*this));
        }

        // 2. Try relative to current working directory
        std::filesystem::path p2(url);
        if (std::filesystem::exists(p2)) {
            return std::make_shared<tesseract::common::SimpleLocatedResource>(
                url, std::filesystem::absolute(p2).string(), std::make_shared<FlexibleResourceLocator>(*this));
        }

        // 3. Try standard fallback (file://, package://, or already absolute)
        return fallback_->locateResource(url);
    }

private:
    std::filesystem::path base_dir_;
    std::shared_ptr<tesseract::common::GeneralResourceLocator> fallback_;
};

// Helper to convert point cloud into Octree AABB boxes for VAMP backend
inline std::vector<double> convertPointCloudToAABBs(const std::vector<double>& points, double resolution, const std::vector<double>& pose) {
    std::vector<double> aabbs;
    if (points.empty()) return aabbs;

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    if (pose.size() >= 7) {
        T.translation() = Eigen::Vector3d(pose[0], pose[1], pose[2]);
        Eigen::Quaterniond q(pose[6], pose[3], pose[4], pose[5]);
        T.linear() = q.matrix();
    }

    tesseract::geometry::PointCloud point_cloud;
    for (size_t i = 0; i + 2 < points.size(); i += 3) {
        Eigen::Vector3d pt(points[i], points[i+1], points[i+2]);
        Eigen::Vector3d w_pt = T * pt;
        point_cloud.addPoint(w_pt.x(), w_pt.y(), w_pt.z());
    }

    auto octree = tesseract::geometry::createOctree(point_cloud, resolution, true, true);
    if (!octree) return aabbs;

    for (auto it = octree->begin_leafs(); it != octree->end_leafs(); ++it) {
        if (octree->isNodeOccupied(*it)) {
            double size = it.getSize();
            double half_size = size * 0.5;
            double x = it.getX();
            double y = it.getY();
            double z = it.getZ();

            aabbs.push_back(x - half_size);
            aabbs.push_back(y - half_size);
            aabbs.push_back(z - half_size);
            aabbs.push_back(x + half_size);
            aabbs.push_back(y + half_size);
            aabbs.push_back(z + half_size);
        }
    }
    return aabbs;
}

// Helper to convert triangular mesh into Octree AABB boxes for VAMP backend
inline std::vector<double> convertMeshToAABBs(const std::vector<double>& vertices,
                                              const std::vector<int>& faces,
                                              const std::vector<double>& pose,
                                              double resolution = 0.04) {
    std::vector<double> aabbs;
    if (vertices.empty() || faces.size() < 3) return aabbs;

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    if (pose.size() >= 7) {
        T.translation() = Eigen::Vector3d(pose[0], pose[1], pose[2]);
        Eigen::Quaterniond q(pose[6], pose[3], pose[4], pose[5]);
        T.linear() = q.matrix();
    }

    tesseract::geometry::PointCloud point_cloud;
    size_t num_triangles = faces.size() / 3;
    double step = resolution * 0.7;

    for (size_t t = 0; t < num_triangles; ++t) {
        int i0 = faces[t * 3 + 0];
        int i1 = faces[t * 3 + 1];
        int i2 = faces[t * 3 + 2];
        if (i0 * 3 + 2 >= (int)vertices.size() || i1 * 3 + 2 >= (int)vertices.size() || i2 * 3 + 2 >= (int)vertices.size()) continue;

        Eigen::Vector3d v0(vertices[i0 * 3 + 0], vertices[i0 * 3 + 1], vertices[i0 * 3 + 2]);
        Eigen::Vector3d v1(vertices[i1 * 3 + 0], vertices[i1 * 3 + 1], vertices[i1 * 3 + 2]);
        Eigen::Vector3d v2(vertices[i2 * 3 + 0], vertices[i2 * 3 + 1], vertices[i2 * 3 + 2]);

        Eigen::Vector3d w_v0 = T * v0;
        Eigen::Vector3d w_v1 = T * v1;
        Eigen::Vector3d w_v2 = T * v2;

        point_cloud.addPoint(w_v0.x(), w_v0.y(), w_v0.z());
        point_cloud.addPoint(w_v1.x(), w_v1.y(), w_v1.z());
        point_cloud.addPoint(w_v2.x(), w_v2.y(), w_v2.z());

        Eigen::Vector3d e1 = w_v1 - w_v0;
        Eigen::Vector3d e2 = w_v2 - w_v0;
        int n1 = std::max(1, static_cast<int>(std::ceil(e1.norm() / step)));
        int n2 = std::max(1, static_cast<int>(std::ceil(e2.norm() / step)));

        for (int u = 0; u <= n1; ++u) {
            double u_frac = static_cast<double>(u) / n1;
            for (int v = 0; v <= n2 - (u * n2 / n1); ++v) {
                double v_frac = static_cast<double>(v) / n2;
                Eigen::Vector3d p = w_v0 + u_frac * e1 + v_frac * e2;
                point_cloud.addPoint(p.x(), p.y(), p.z());
            }
        }
    }

    auto octree = tesseract::geometry::createOctree(point_cloud, resolution, true, true);
    if (!octree) return aabbs;

    for (auto it = octree->begin_leafs(); it != octree->end_leafs(); ++it) {
        if (octree->isNodeOccupied(*it)) {
            double size = it.getSize();
            double half_size = size * 0.5;
            double x = it.getX();
            double y = it.getY();
            double z = it.getZ();

            aabbs.push_back(x - half_size);
            aabbs.push_back(y - half_size);
            aabbs.push_back(z - half_size);
            aabbs.push_back(x + half_size);
            aabbs.push_back(y + half_size);
            aabbs.push_back(z + half_size);
        }
    }
    return aabbs;
}

} // namespace robot_planner
