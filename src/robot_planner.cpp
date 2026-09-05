#include <robot_planner/robot_planner.h>

#include <iostream>
#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/environment/environment.h>
#include <tesseract/environment/utils.h>
#include <tesseract/environment/commands/add_link_command.h>
#include <tesseract/environment/commands/remove_link_command.h>
#include <tesseract/scene_graph/link.h>
#include <tesseract/scene_graph/joint.h>
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

namespace robot_planner {

struct RobotPlanner::Impl {
    std::shared_ptr<tesseract::environment::Environment> env_;
    std::string manipulator_name_;
    std::string base_link_;
    std::string tool_link_;
    
    // Task composer factory for planning
    std::unique_ptr<tesseract::task_composer::TaskComposerPluginFactory> factory_;
};

RobotPlanner::RobotPlanner() : pimpl_(std::make_unique<Impl>()) {
}

RobotPlanner::~RobotPlanner() = default;

bool RobotPlanner::init(const std::string& urdf_path, const std::string& srdf_path, 
                        const std::string& manipulator_name, 
                        const std::string& base_link, 
                        const std::string& tool_link) {
    pimpl_->env_ = std::make_shared<tesseract::environment::Environment>();
    auto locator = std::make_shared<tesseract::common::GeneralResourceLocator>();
    
    // You might want to register the paths. 
    // Here we assume urdf_path and srdf_path are absolute file paths or valid URIs.
    if (!pimpl_->env_->init(std::filesystem::path(urdf_path), std::filesystem::path(srdf_path), locator)) {
        std::cerr << "Failed to initialize environment." << std::endl;
        return false;
    }
    
    pimpl_->manipulator_name_ = manipulator_name;
    pimpl_->base_link_ = base_link;
    pimpl_->tool_link_ = tool_link;
    
    // Initialize task composer factory
    std::filesystem::path config_path(locator->locateResource("package://tesseract_planning/task_composer/config/task_composer_plugins.yaml")->getFilePath());
    if (std::filesystem::exists(config_path)) {
        pimpl_->factory_ = std::make_unique<tesseract::task_composer::TaskComposerPluginFactory>(config_path, *locator);
    } else {
        std::cerr << "Warning: Task composer plugin config not found at " << config_path << std::endl;
    }
    
    return true;
}

bool RobotPlanner::computeFK(const std::vector<double>& joint_angles, std::vector<double>& pose_out) {
    if (!pimpl_->env_) return false;
    auto joint_group = pimpl_->env_->getJointGroup(pimpl_->manipulator_name_);
    if (!joint_group) {
        std::cerr << "No joint group found for manipulator: " << pimpl_->manipulator_name_ << std::endl;
        return false;
    }
    
    if (joint_angles.size() != joint_group->getJointNames().size()) {
        std::cerr << "computeFK: joint_angles size (" << joint_angles.size() << ") does not match joint group size (" << joint_group->getJointNames().size() << ")." << std::endl;
        return false;
    }
    Eigen::VectorXd joints = Eigen::Map<const Eigen::VectorXd>(joint_angles.data(), joint_angles.size());
    auto transforms = joint_group->calcFwdKin(joints);
    
    if (transforms.find(pimpl_->tool_link_) == transforms.end()) {
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
    
    return true;
}

bool RobotPlanner::computeIK(const std::vector<double>& pose, const std::vector<double>& seed_joint_angles, std::vector<double>& joint_angles_out) {
    if (!pimpl_->env_) return false;
    auto kin_group = pimpl_->env_->getKinematicGroup(pimpl_->manipulator_name_);
    if (!kin_group) {
        std::cerr << "No kinematic group found for " << pimpl_->manipulator_name_ << std::endl;
        return false;
    }
    
    if (pose.size() < 7) {
        std::cerr << "computeIK: target pose size must be at least 7 (x,y,z,qx,qy,qz,qw)." << std::endl;
        return false;
    }
    if (seed_joint_angles.size() != kin_group->getJointNames().size()) {
        std::cerr << "computeIK: seed_joint_angles size (" << seed_joint_angles.size() << ") does not match kin group size (" << kin_group->getJointNames().size() << ")." << std::endl;
        return false;
    }
    
    Eigen::Isometry3d target_pose = Eigen::Isometry3d::Identity();
    target_pose.translation() = Eigen::Vector3d(pose[0], pose[1], pose[2]);
    Eigen::Quaterniond q(pose[6], pose[3], pose[4], pose[5]); // w, x, y, z
    target_pose.linear() = q.matrix();
    
    Eigen::VectorXd seed = Eigen::Map<const Eigen::VectorXd>(seed_joint_angles.data(), seed_joint_angles.size());
    tesseract::kinematics::KinGroupIKInput ik_input(target_pose, pimpl_->base_link_, pimpl_->tool_link_);
    tesseract::kinematics::IKSolutions solutions = kin_group->calcInvKin(ik_input, seed);
    
    if (solutions.empty()) {
        return false;
    }
    
    Eigen::VectorXd solution = solutions[0];
    
    joint_angles_out.resize(solution.size());
    for (int i=0; i<solution.size(); ++i) {
        joint_angles_out[i] = solution(i);
    }
    return true;
}

bool RobotPlanner::checkCollision(const std::vector<double>& joint_angles) {
    if (!pimpl_->env_) return false;
    
    auto active_link_names = pimpl_->env_->getActiveLinkNames();
    std::vector<std::string> joint_names = pimpl_->env_->getJointGroup(pimpl_->manipulator_name_)->getJointNames();
    
    if (joint_angles.size() != joint_names.size()) {
        std::cerr << "checkCollision: joint_angles size (" << joint_angles.size() << ") does not match joint group size (" << joint_names.size() << ")." << std::endl;
        return false;
    }
    Eigen::VectorXd joints = Eigen::Map<const Eigen::VectorXd>(joint_angles.data(), joint_angles.size());
    
    // Set state
    tesseract::scene_graph::SceneState state = pimpl_->env_->getState(joint_names, joints);
    
    // Check collision
    tesseract::collision::DiscreteContactManager::Ptr manager = pimpl_->env_->getDiscreteContactManager();
    manager->setActiveCollisionObjects(active_link_names);
    manager->setCollisionObjectsTransform(state.link_transforms);
    
    tesseract::collision::ContactResultMap contact_results;
    manager->contactTest(contact_results, tesseract::collision::ContactTestType::FIRST);
    
    return !contact_results.empty();
}

bool RobotPlanner::addBox(const std::string& name, double x, double y, double z, double dim_x, double dim_y, double dim_z) {
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
    return pimpl_->env_->applyCommand(cmd);
}

bool RobotPlanner::addMesh(const std::string& name,
                           const std::vector<double>& vertices,
                           const std::vector<int>& faces,
                           const std::vector<double>& pose) {
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
    return pimpl_->env_->applyCommand(cmd);
}

bool RobotPlanner::addPointCloud(const std::string& name,
                                 const std::vector<double>& points,
                                 double resolution,
                                 const std::vector<double>& pose) {
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
    return pimpl_->env_->applyCommand(cmd);
}

bool RobotPlanner::removeObstacle(const std::string& name) {
    if (!pimpl_->env_) return false;
    auto cmd = std::make_shared<tesseract::environment::RemoveLinkCommand>(name);
    return pimpl_->env_->applyCommand(cmd);
}

bool RobotPlanner::planFreespace(const std::vector<double>& start_joints, 
                                 const std::vector<double>& target_joints, 
                                 std::vector<std::vector<double>>& trajectory_out,
                                 double planning_time,
                                 double range,
                                 double safety_margin,
                                 double collision_coeff,
                                 const std::string& planner_type) {
    if (!pimpl_->env_ || !pimpl_->factory_) return false;
    
    std::vector<std::string> joint_names = pimpl_->env_->getJointGroup(pimpl_->manipulator_name_)->getJointNames();
    if (start_joints.size() != joint_names.size()) {
        std::cerr << "planFreespace: start_joints size mismatch." << std::endl;
        return false;
    }
    if (target_joints.size() != joint_names.size()) {
        std::cerr << "planFreespace: target_joints size mismatch." << std::endl;
        return false;
    }
    tesseract::common::ManipulatorInfo manip_info(pimpl_->manipulator_name_, pimpl_->base_link_, pimpl_->tool_link_);
    tesseract::command_language::CompositeInstruction program("FREESPACE", manip_info);
    
    Eigen::VectorXd start = Eigen::Map<const Eigen::VectorXd>(start_joints.data(), start_joints.size());
    Eigen::VectorXd target = Eigen::Map<const Eigen::VectorXd>(target_joints.data(), target_joints.size());
    
    tesseract::command_language::StateWaypoint wp0(joint_names, start);
    tesseract::command_language::MoveInstruction start_inst(wp0, tesseract::command_language::MoveInstructionType::FREESPACE, "FREESPACE");
    start_inst.setDescription("Start Instruction");
    
    tesseract::command_language::StateWaypoint wp1(joint_names, target);
    tesseract::command_language::MoveInstruction plan_inst(wp1, tesseract::command_language::MoveInstructionType::FREESPACE, "FREESPACE");
    plan_inst.setDescription("freespace_plan");
    
    program.push_back(start_inst);
    program.push_back(plan_inst);
    
    // Create profile dictionary
    auto profiles = std::make_shared<tesseract::common::ProfileDictionary>();
    
    // 1. OMPL Profile
    auto ompl_profile = std::make_shared<tesseract::motion_planners::OMPLRealVectorMoveProfile>();
    std::shared_ptr<tesseract::motion_planners::OMPLPlannerConfigurator> ompl_planner_config;
    
    if (planner_type == "RRTstar" || planner_type == "RRT*" || planner_type == "rrtstar") {
        auto rrtstar = std::make_shared<tesseract::motion_planners::RRTstarConfigurator>();
        rrtstar->range = range;
        ompl_planner_config = rrtstar;
    } else if (planner_type == "PRM" || planner_type == "prm") {
        auto prm = std::make_shared<tesseract::motion_planners::PRMConfigurator>();
        ompl_planner_config = prm;
    } else {
        auto rrtconnect = std::make_shared<tesseract::motion_planners::RRTConnectConfigurator>();
        rrtconnect->range = range;
        ompl_planner_config = rrtconnect;
    }
    
    ompl_profile->solver_config.planning_time = planning_time;
    ompl_profile->solver_config.planners = { ompl_planner_config, ompl_planner_config };
    profiles->addProfile("OMPLMotionPlannerTask", "FREESPACE", ompl_profile);
    
    // 2. TrajOpt Profile (用于优化平滑并避障)
    auto trajopt_composite_profile = std::make_shared<tesseract::motion_planners::TrajOptDefaultCompositeProfile>();
    trajopt_composite_profile->collision_cost_config = trajopt_common::TrajOptCollisionConfig(safety_margin, collision_coeff);
    trajopt_composite_profile->collision_cost_config.enabled = true;
    profiles->addProfile("TrajOptMotionPlannerTask", "FREESPACE", trajopt_composite_profile);
    
    auto trajopt_move_profile = std::make_shared<tesseract::motion_planners::TrajOptDefaultMoveProfile>();
    profiles->addProfile("TrajOptMotionPlannerTask", "FREESPACE", trajopt_move_profile);
    
    auto task = pimpl_->factory_->createTaskComposerNode("FreespacePipeline");
    if (!task) return false;
    
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
        
        trajectory_out.clear();
        for (const auto& wp : traj) {
            std::vector<double> point(wp.position.data(), wp.position.data() + wp.position.size());
            trajectory_out.push_back(point);
        }
        return true;
    }
    return false;
}

bool RobotPlanner::planLinear(const std::vector<double>& start_joints, 
                              const std::vector<double>& target_pose, 
                              std::vector<std::vector<double>>& trajectory_out,
                              double safety_margin,
                              double collision_coeff) {
    if (!pimpl_->env_ || !pimpl_->factory_) return false;
    
    std::vector<std::string> joint_names = pimpl_->env_->getJointGroup(pimpl_->manipulator_name_)->getJointNames();
    if (start_joints.size() != joint_names.size()) {
        std::cerr << "planLinear: start_joints size mismatch." << std::endl;
        return false;
    }
    if (target_pose.size() < 7) {
        std::cerr << "planLinear: target_pose size must be at least 7." << std::endl;
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
    if (!task) return false;
    
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
        
        trajectory_out.clear();
        for (const auto& wp : traj) {
            std::vector<double> point(wp.position.data(), wp.position.data() + wp.position.size());
            trajectory_out.push_back(point);
        }
        return true;
    }
    return false;
}

bool RobotPlanner::planCircular(const std::vector<double>& start_joints, 
                                const std::vector<double>& aux_pose, 
                                const std::vector<double>& target_pose, 
                                std::vector<std::vector<double>>& trajectory_out,
                                double safety_margin,
                                double collision_coeff) {
    if (!pimpl_->env_ || !pimpl_->factory_) return false;
    
    std::vector<std::string> joint_names = pimpl_->env_->getJointGroup(pimpl_->manipulator_name_)->getJointNames();
    if (start_joints.size() != joint_names.size()) {
        std::cerr << "planCircular: start_joints size mismatch." << std::endl;
        return false;
    }
    if (aux_pose.size() < 7 || target_pose.size() < 7) {
        std::cerr << "planCircular: pose size must be at least 7." << std::endl;
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
    if (!task) return false;
    
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
        
        trajectory_out.clear();
        for (const auto& wp : traj) {
            std::vector<double> point(wp.position.data(), wp.position.data() + wp.position.size());
            trajectory_out.push_back(point);
        }
        return true;
    }
    return false;
}

} // namespace robot_planner
