#include "planner_impl.h"

namespace robot_planner {

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
            pimpl_->vamp_loader_->setLimits(pimpl_->joint_vel_limits_, pimpl_->joint_acc_limits_);

            if (!pimpl_->vamp_loader_->loadSRDF(srdf_path)) {
                std::cerr << "[RobotPlanner] Error: VAMP failed to load/parse SRDF: " << srdf_path << std::endl;
                pimpl_->setLastError(PlannerStatus::INVALID_ARGUMENTS, "Failed to load/parse SRDF in VAMP: " + srdf_path);
                return false;
            }

            std::cout << "[RobotPlanner] VAMP Backend activated (AVX2 SIMD, URDF dynamic joint origins injected, SRDF ACM loaded) -> "
                      << pimpl_->vamp_loader_->getResolvedPath() << std::endl;
        } else {
            if (backend == PlannerBackend::VAMP) {
                pimpl_->setLastError(PlannerStatus::NOT_INITIALIZED, "VAMP plugin could not be loaded.");
                std::cerr << "[RobotPlanner] Error: VAMP plugin could not be loaded." << std::endl;
                return false;
            }
        }
    }

    if (pimpl_->backend_ == PlannerBackend::TESSERACT) {
        std::cout << "[RobotPlanner] TESSERACT Backend activated (Bullet/FCL DiscreteContactManager)." << std::endl;
    }

    pimpl_->setLastError(PlannerStatus::SUCCESS, "");
    return true;
}

} // namespace robot_planner
