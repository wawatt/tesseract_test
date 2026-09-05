#include <robot_planner/robot_planner.h>

#include <iostream>
#include <unordered_set>
#include <unordered_map>
#include <cmath>
#include <chrono>
#include <algorithm>

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

struct RobotPlanner::Impl {
    std::shared_ptr<tesseract::environment::Environment> env_;
    std::string manipulator_name_;
    std::string base_link_;
    std::string tool_link_;
    
    // Task composer factory for planning
    std::unique_ptr<tesseract::task_composer::TaskComposerPluginFactory> factory_;

    // Optional dynamic loader for vamp_r2000ic.dll
    std::unique_ptr<VampDynamicLoader> vamp_loader_;
    PlannerBackend backend_ = PlannerBackend::TESSERACT;

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
};

RobotPlanner::RobotPlanner() : pimpl_(std::make_unique<Impl>()) {
}

RobotPlanner::~RobotPlanner() = default;

PlannerBackend RobotPlanner::getBackend() const {
    return pimpl_->backend_;
}

PlannerStatus RobotPlanner::getLastErrorStatus() const {
    return pimpl_->last_status_;
}

std::string RobotPlanner::getLastError() const {
    return pimpl_->last_error_;
}

namespace {

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

std::vector<double> convertPointCloudToAABBs(const std::vector<double>& points, double resolution, const std::vector<double>& pose) {
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

std::vector<double> convertMeshToAABBs(const std::vector<double>& vertices,
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

} // namespace

bool RobotPlanner::init(const std::string& urdf_path, const std::string& srdf_path, 
                        const std::string& manipulator_name, 
                        const std::string& base_link, 
                        const std::string& tool_link,
                        PlannerBackend backend,
                        const std::string& custom_plugin_path) {
    pimpl_->env_ = std::make_shared<tesseract::environment::Environment>();

    std::filesystem::path urdf_f(urdf_path);
    std::filesystem::path base_dir = urdf_f.has_parent_path() ? urdf_f.parent_path() : std::filesystem::current_path();
    auto locator = std::make_shared<FlexibleResourceLocator>(base_dir);
    
    if (!pimpl_->env_->init(std::filesystem::path(urdf_path), std::filesystem::path(srdf_path), locator)) {
        pimpl_->setLastError(PlannerStatus::NOT_INITIALIZED, "Failed to initialize Tesseract Environment from URDF/SRDF.");
        std::cerr << "Failed to initialize environment." << std::endl;
        return false;
    }
    
    pimpl_->manipulator_name_ = manipulator_name;
    pimpl_->base_link_ = base_link;
    pimpl_->tool_link_ = tool_link;

    // -------------------------------------------------------------
    // 从已解析的 URDF 提取 6 个关节的 origin xyz/rpy 与 axis 及物理限位
    // -------------------------------------------------------------
    pimpl_->joint_origins_xyz_.clear();
    pimpl_->joint_origins_rpy_.clear();
    pimpl_->joint_axes_xyz_.clear();
    pimpl_->joint_limits_min_.clear();
    pimpl_->joint_limits_max_.clear();
    pimpl_->joint_vel_limits_.clear();
    pimpl_->joint_acc_limits_.clear();

    auto joint_group = pimpl_->env_->getJointGroup(manipulator_name);
    if (!joint_group) {
        pimpl_->setLastError(PlannerStatus::INVALID_ARGUMENTS, "Joint group '" + manipulator_name + "' not found in SRDF.");
        return false;
    }

    auto scene_graph = pimpl_->env_->getSceneGraph();
    std::vector<std::string> joint_names = joint_group->getJointNames();

    for (const auto& jname : joint_names) {
        auto joint = scene_graph->getJoint(jname);
        if (!joint) {
            pimpl_->setLastError(PlannerStatus::INTERNAL_ERROR, "Joint '" + jname + "' not found in SceneGraph.");
            return false;
        }

        // Translation
        Eigen::Vector3d trans = joint->parent_to_joint_origin_transform.translation();
        pimpl_->joint_origins_xyz_.push_back(trans.x());
        pimpl_->joint_origins_xyz_.push_back(trans.y());
        pimpl_->joint_origins_xyz_.push_back(trans.z());

        // Rotation RPY
        Eigen::Matrix3d R = joint->parent_to_joint_origin_transform.rotation();
        Eigen::Vector3d rpy = R.eulerAngles(0, 1, 2); // Roll, Pitch, Yaw
        pimpl_->joint_origins_rpy_.push_back(rpy[0]);
        pimpl_->joint_origins_rpy_.push_back(rpy[1]);
        pimpl_->joint_origins_rpy_.push_back(rpy[2]);

        // Joint rotation axis
        Eigen::Vector3d axis = joint->axis;
        pimpl_->joint_axes_xyz_.push_back(axis.x());
        pimpl_->joint_axes_xyz_.push_back(axis.y());
        pimpl_->joint_axes_xyz_.push_back(axis.z());

        // Joint limits
        if (joint->limits) {
            pimpl_->joint_limits_min_.push_back(joint->limits->lower);
            pimpl_->joint_limits_max_.push_back(joint->limits->upper);
            pimpl_->joint_vel_limits_.push_back(joint->limits->velocity > 0.0 ? joint->limits->velocity : 2.0);
            pimpl_->joint_acc_limits_.push_back(joint->limits->acceleration > 0.0 ? joint->limits->acceleration : 5.0);
        } else {
            pimpl_->joint_limits_min_.push_back(-M_PI);
            pimpl_->joint_limits_max_.push_back(M_PI);
            pimpl_->joint_vel_limits_.push_back(2.0);
            pimpl_->joint_acc_limits_.push_back(5.0);
        }
    }

    pimpl_->opw_params_ = OpwKinematics::fromJointOrigins(pimpl_->joint_origins_xyz_);
    std::cout << "[RobotPlanner] Successfully extracted 6 joint kinematics & limits from URDF." << std::endl;
    
    // Initialize task composer factory
    std::filesystem::path config_path(locator->locateResource("package://tesseract_planning/task_composer/config/task_composer_plugins.yaml")->getFilePath());
    if (std::filesystem::exists(config_path)) {
        pimpl_->factory_ = std::make_unique<tesseract::task_composer::TaskComposerPluginFactory>(config_path, *locator);
    } else {
        std::cerr << "Warning: Task composer plugin config not found at " << config_path << std::endl;
    }
    
    // Configure Backend
    pimpl_->backend_ = PlannerBackend::TESSERACT; // Default fallback

    bool should_try_vamp = (backend == PlannerBackend::VAMP || backend == PlannerBackend::AUTO);

    if (should_try_vamp) {
        pimpl_->vamp_loader_ = std::make_unique<VampDynamicLoader>();
        std::string robot_name = manipulator_name;
        if (pimpl_->vamp_loader_->load(custom_plugin_path, robot_name)) {
            pimpl_->backend_ = PlannerBackend::VAMP;
            pimpl_->vamp_loader_->setJointOrigins(pimpl_->joint_origins_xyz_, pimpl_->joint_origins_rpy_, pimpl_->joint_axes_xyz_);
            std::cout << "[RobotPlanner] VAMP Backend activated (AVX2 SIMD, URDF dynamic joint origins injected) -> "
                      << pimpl_->vamp_loader_->getResolvedPath() << std::endl;
        } else {
            if (backend == PlannerBackend::VAMP) {
                std::cout << "[RobotPlanner] Notice: VAMP plugin could not be loaded. "
                          << "Falling back to TESSERACT Backend." << std::endl;
            }
        }
    }

    if (pimpl_->backend_ == PlannerBackend::TESSERACT) {
        std::cout << "[RobotPlanner] TESSERACT Backend activated (Bullet/FCL DiscreteContactManager)." << std::endl;
    }

    pimpl_->setLastError(PlannerStatus::SUCCESS, "");
    return true;
}

bool RobotPlanner::computeFK(const std::vector<double>& joint_angles, std::vector<double>& pose_out) {
    if (!pimpl_->env_) {
        pimpl_->setLastError(PlannerStatus::NOT_INITIALIZED, "Environment not initialized.");
        return false;
    }
    auto joint_group = pimpl_->env_->getJointGroup(pimpl_->manipulator_name_);
    if (!joint_group) {
        pimpl_->setLastError(PlannerStatus::INTERNAL_ERROR, "No joint group found for manipulator.");
        return false;
    }
    
    if (joint_angles.size() != joint_group->getJointNames().size()) {
        pimpl_->setLastError(PlannerStatus::INVALID_ARGUMENTS, "Joint angles size mismatch.");
        return false;
    }

    Eigen::VectorXd joints = Eigen::Map<const Eigen::VectorXd>(joint_angles.data(), joint_angles.size());
    auto transforms = joint_group->calcFwdKin(joints);
    
    if (transforms.find(pimpl_->tool_link_) == transforms.end()) {
        pimpl_->setLastError(PlannerStatus::INTERNAL_ERROR, "Tool link '" + pimpl_->tool_link_ + "' not found in FK transforms.");
        return false;
    }
    Eigen::Isometry3d pose = transforms.at(pimpl_->tool_link_);
    
    pose_out.resize(7);
    pose_out[0] = pose.translation().x();
    pose_out[1] = pose.translation().y();
    pose_out[2] = pose.translation().z();
    Eigen::Quaterniond q(pose.rotation());
    pose_out[3] = q.x();
    pose_out[4] = q.y();
    pose_out[5] = q.z();
    pose_out[6] = q.w();
    
    pimpl_->setLastError(PlannerStatus::SUCCESS, "");
    return true;
}

bool RobotPlanner::computeAllIK(const std::vector<double>& pose, std::vector<std::vector<double>>& all_solutions_out) {
    all_solutions_out.clear();
    if (pose.size() < 7) {
        pimpl_->setLastError(PlannerStatus::INVALID_ARGUMENTS, "Target pose size must be at least 7.");
        return false;
    }

    std::vector<std::vector<double>> raw_solutions;
    if (!OpwKinematics::computeAllIK(pimpl_->opw_params_, pose, raw_solutions)) {
        pimpl_->setLastError(PlannerStatus::IK_FAILED, "No mathematical analytical inverse kinematics solution exists for pose.");
        return false;
    }

    // Filter against URDF physical joint limits (with 2*pi periodicity normalization)
    for (auto sol : raw_solutions) {
        bool in_limits = true;
        if (sol.size() == pimpl_->joint_limits_min_.size()) {
            for (size_t j = 0; j < sol.size(); ++j) {
                double min_limit = pimpl_->joint_limits_min_[j] - 1e-4;
                double max_limit = pimpl_->joint_limits_max_[j] + 1e-4;
                while (sol[j] < min_limit && (sol[j] + 2.0 * M_PI) <= max_limit) {
                    sol[j] += 2.0 * M_PI;
                }
                while (sol[j] > max_limit && (sol[j] - 2.0 * M_PI) >= min_limit) {
                    sol[j] -= 2.0 * M_PI;
                }
                if (sol[j] < min_limit || sol[j] > max_limit) {
                    in_limits = false;
                    break;
                }
            }
        }
        if (in_limits) {
            all_solutions_out.push_back(sol);
        }
    }

    if (all_solutions_out.empty()) {
        pimpl_->setLastError(PlannerStatus::JOINT_LIMIT_VIOLATED, "Analytical IK solutions exist but all violate URDF physical joint limits.");
        return false;
    }

    pimpl_->setLastError(PlannerStatus::SUCCESS, "");
    return true;
}

bool RobotPlanner::computeIK(const std::vector<double>& pose, const std::vector<double>& seed_joint_angles, std::vector<double>& joint_angles_out) {
    if (pose.size() < 7) {
        pimpl_->setLastError(PlannerStatus::INVALID_ARGUMENTS, "Target pose size must be at least 7.");
        return false;
    }

    // 1. Try OPW analytical inverse kinematics with limits check
    std::vector<std::vector<double>> valid_solutions;
    if (computeAllIK(pose, valid_solutions) && !valid_solutions.empty()) {
        double min_dist_sq = std::numeric_limits<double>::max();
        size_t best_idx = 0;
        for (size_t i = 0; i < valid_solutions.size(); ++i) {
            double dist_sq = 0.0;
            for (size_t j = 0; j < valid_solutions[i].size(); ++j) {
                double diff = valid_solutions[i][j] - (j < seed_joint_angles.size() ? seed_joint_angles[j] : 0.0);
                dist_sq += diff * diff;
            }
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                best_idx = i;
            }
        }
        joint_angles_out = valid_solutions[best_idx];
        pimpl_->setLastError(PlannerStatus::SUCCESS, "");
        return true;
    }

    // 2. Fallback to Tesseract numerical kinematics solver
    if (!pimpl_->env_) {
        pimpl_->setLastError(PlannerStatus::NOT_INITIALIZED, "Environment not initialized.");
        return false;
    }
    try {
        auto kin_group = pimpl_->env_->getKinematicGroup(pimpl_->manipulator_name_);
        if (!kin_group) {
            pimpl_->setLastError(PlannerStatus::INTERNAL_ERROR, "No kinematic group found for " + pimpl_->manipulator_name_);
            return false;
        }
        
        Eigen::Isometry3d target_pose = Eigen::Isometry3d::Identity();
        target_pose.translation() = Eigen::Vector3d(pose[0], pose[1], pose[2]);
        Eigen::Quaterniond q(pose[6], pose[3], pose[4], pose[5]);
        target_pose.linear() = q.matrix();
        
        Eigen::VectorXd seed = Eigen::Map<const Eigen::VectorXd>(seed_joint_angles.data(), seed_joint_angles.size());
        tesseract::kinematics::KinGroupIKInput ik_input(target_pose, pimpl_->base_link_, pimpl_->tool_link_);
        tesseract::kinematics::IKSolutions solutions = kin_group->calcInvKin(ik_input, seed);
        
        if (solutions.empty()) {
            pimpl_->setLastError(PlannerStatus::IK_FAILED, "Numerical IK found no solution.");
            return false;
        }
        
        Eigen::VectorXd solution = solutions[0];
        joint_angles_out.resize(solution.size());
        for (int i = 0; i < solution.size(); ++i) {
            joint_angles_out[i] = solution(i);
        }
        pimpl_->setLastError(PlannerStatus::SUCCESS, "");
        return true;
    } catch (const std::exception& e) {
        pimpl_->setLastError(PlannerStatus::INTERNAL_ERROR, std::string("computeIK exception: ") + e.what());
        return false;
    }
}

bool RobotPlanner::checkCollision(const std::vector<double>& joint_angles) {
    if (pimpl_->backend_ == PlannerBackend::VAMP && pimpl_->vamp_loader_ && joint_angles.size() == 6) {
        bool col = pimpl_->vamp_loader_->checkCollision(joint_angles);
        if (col) {
            pimpl_->setLastError(PlannerStatus::COLLISION_DETECTED, "Collision detected by VAMP SIMD engine.");
        } else {
            pimpl_->setLastError(PlannerStatus::SUCCESS, "");
        }
        return col;
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
    Eigen::VectorXd joints = Eigen::Map<const Eigen::VectorXd>(joint_angles.data(), joint_angles.size());
    
    tesseract::scene_graph::SceneState state = pimpl_->env_->getState(joint_names, joints);
    
    tesseract::collision::DiscreteContactManager::Ptr manager = pimpl_->env_->getDiscreteContactManager();
    manager->setActiveCollisionObjects(active_link_names);
    manager->setCollisionObjectsTransform(state.link_transforms);
    
    tesseract::collision::ContactResultMap contact_results;
    manager->contactTest(contact_results, tesseract::collision::ContactTestType::FIRST);
    
    bool col = !contact_results.empty();
    if (col) {
        pimpl_->setLastError(PlannerStatus::COLLISION_DETECTED, "Collision detected by Tesseract DiscreteContactManager.");
    } else {
        pimpl_->setLastError(PlannerStatus::SUCCESS, "");
    }
    return col;
}

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

bool RobotPlanner::addMesh(const std::string& name,
                           const std::vector<double>& vertices,
                           const std::vector<int>& faces,
                           const std::vector<double>& pose) {
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

bool RobotPlanner::removeObstacle(const std::string& name) {
    if (pimpl_->vamp_loader_ && pimpl_->vamp_loader_->isLoaded()) {
        pimpl_->vamp_loader_->removeObstacle(name);
    }
    pimpl_->attached_obstacles_.erase(name);
    pimpl_->obstacle_names_.erase(name);

    if (!pimpl_->env_) return false;
    auto cmd = std::make_shared<tesseract::environment::RemoveLinkCommand>(name);
    bool ok = pimpl_->env_->applyCommand(cmd);
    pimpl_->setLastError(ok ? PlannerStatus::SUCCESS : PlannerStatus::OBSTACLE_NOT_FOUND, ok ? "" : "Failed to remove link from Tesseract.");
    return ok;
}

bool RobotPlanner::clearObstacles() {
    auto names = getObstacleNames();
    bool all_ok = true;
    for (const auto& name : names) {
        if (!removeObstacle(name)) {
            all_ok = false;
        }
    }
    pimpl_->setLastError(all_ok ? PlannerStatus::SUCCESS : PlannerStatus::INTERNAL_ERROR, all_ok ? "" : "Failed to clear all obstacles.");
    return all_ok;
}

std::vector<std::string> RobotPlanner::getObstacleNames() const {
    return std::vector<std::string>(pimpl_->obstacle_names_.begin(), pimpl_->obstacle_names_.end());
}

bool RobotPlanner::hasObstacle(const std::string& name) const {
    return pimpl_->obstacle_names_.find(name) != pimpl_->obstacle_names_.end();
}

bool RobotPlanner::attachObject(const std::string& obstacle_name, const std::string& link_name) {
    if (!hasObstacle(obstacle_name)) {
        pimpl_->setLastError(PlannerStatus::OBSTACLE_NOT_FOUND, "Obstacle '" + obstacle_name + "' not found in scene.");
        return false;
    }
    std::string target_link = link_name.empty() ? pimpl_->tool_link_ : link_name;
    if (!pimpl_->env_) {
        pimpl_->setLastError(PlannerStatus::NOT_INITIALIZED, "Environment not initialized.");
        return false;
    }

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
        } else if (planner_type == "PRM" || planner_type == "prm") {
            ompl_planner_config = std::make_shared<tesseract::motion_planners::PRMConfigurator>();
        } else {
            auto rrtconnect = std::make_shared<tesseract::motion_planners::RRTConnectConfigurator>();
            rrtconnect->range = range;
            ompl_planner_config = rrtconnect;
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

        std::vector<std::vector<double>> seed_traj;
        seed_traj.push_back(start_joints);
        std::vector<double> prev_joints = start_joints;

        for (int k = 1; k <= steps; ++k) {
            double t = static_cast<double>(k) / steps;
            Eigen::Vector3d p_t = (1.0 - t) * p_start + t * p_target;
            Eigen::Quaterniond q_t = q_start.slerp(t, q_target);

            std::vector<double> step_pose = { p_t.x(), p_t.y(), p_t.z(), q_t.x(), q_t.y(), q_t.z(), q_t.w() };
            std::vector<double> step_joints;
            if (!computeIK(step_pose, prev_joints, step_joints)) {
                pimpl_->setLastError(PlannerStatus::IK_FAILED, "Linear IK failed at step " + std::to_string(k) + "/" + std::to_string(steps) + " (" + pimpl_->last_error_ + ")");
                return false;
            }

            // Singularity & Axis jump protection
            for (size_t j = 0; j < step_joints.size(); ++j) {
                if (std::abs(step_joints[j] - prev_joints[j]) > 0.8) {
                    pimpl_->setLastError(PlannerStatus::SINGULARITY_DETECTED, "Joint jump / axis flip detected on axis " + std::to_string(j + 1));
                    return false;
                }
            }

            if (checkCollision(step_joints)) {
                pimpl_->setLastError(PlannerStatus::COLLISION_DETECTED, "Linear path collides with obstacle at step " + std::to_string(k));
                return false;
            }

            seed_traj.push_back(step_joints);
            prev_joints = step_joints;
        }

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

    // VAMP accelerated mode (OPW IK + SIMD collision + TrajOpt parameterization)
    if (pimpl_->backend_ == PlannerBackend::VAMP && pimpl_->vamp_loader_ && start_joints.size() == 6) {
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

        std::vector<std::vector<double>> seed_traj;
        seed_traj.push_back(start_joints);
        std::vector<double> prev_joints = start_joints;

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
            std::vector<double> step_joints;
            if (!computeIK(step_pose, prev_joints, step_joints)) {
                pimpl_->setLastError(PlannerStatus::IK_FAILED, "Circular IK failed at step " + std::to_string(k) + "/" + std::to_string(steps) + " (" + pimpl_->last_error_ + ")");
                return false;
            }

            // Singularity / axis flip protection
            for (size_t j = 0; j < step_joints.size(); ++j) {
                if (std::abs(step_joints[j] - prev_joints[j]) > 0.8) {
                    pimpl_->setLastError(PlannerStatus::SINGULARITY_DETECTED, "Joint jump / axis flip detected on axis " + std::to_string(j + 1));
                    return false;
                }
            }

            if (checkCollision(step_joints)) {
                pimpl_->setLastError(PlannerStatus::COLLISION_DETECTED, "Circular path collides with obstacle at step " + std::to_string(k));
                return false;
            }

            seed_traj.push_back(step_joints);
            prev_joints = step_joints;
        }

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
