#include "vamp_r2000ic/vamp_r2000ic_planner.hpp"
#include "vamp_r2000ic/vamp_ompl_adapter.hpp"

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/PathSimplifier.h>

namespace vamp_r2000ic {

struct VampR2000icPlanner::Impl {
    CollisionChecker checker_;
    std::shared_ptr<ompl::base::RealVectorStateSpace> space_;
    ompl::base::SpaceInformationPtr si_;

    bool initialized_ = false;
};

VampR2000icPlanner::VampR2000icPlanner() : pimpl_(std::make_unique<Impl>()) {}
VampR2000icPlanner::~VampR2000icPlanner() = default;

bool VampR2000icPlanner::init() {
    pimpl_->space_ = createR2000icStateSpace();
    pimpl_->si_ = std::make_shared<ompl::base::SpaceInformation>(pimpl_->space_);
    pimpl_->initialized_ = true;
    return true;
}

void VampR2000icPlanner::setJointOrigins(const double origins_xyz[18], const double origins_rpy[18], const double axes_xyz[18]) {
    pimpl_->checker_.setJointOrigins(origins_xyz, origins_rpy, axes_xyz);
}

void VampR2000icPlanner::addObstacleBox(const std::string& name, double cx, double cy, double cz,
                                       double dx, double dy, double dz) {
    pimpl_->checker_.addBoxCenterDim(name, static_cast<float>(cx), static_cast<float>(cy), static_cast<float>(cz),
                                     static_cast<float>(dx), static_cast<float>(dy), static_cast<float>(dz));
}

void VampR2000icPlanner::addObstacleBoxes(const std::string& name, const std::vector<AABB>& boxes) {
    pimpl_->checker_.addBoxes(name, boxes);
}

bool VampR2000icPlanner::removeObstacle(const std::string& name) {
    return pimpl_->checker_.removeObstacle(name);
}

void VampR2000icPlanner::clearObstacles() {
    pimpl_->checker_.clearObstacles();
}

bool VampR2000icPlanner::checkCollision(const std::vector<double>& joints, double safety_margin) {
    if (joints.size() < 6) return true;
    return pimpl_->checker_.checkCollision(joints.data(), static_cast<float>(safety_margin));
}

CollisionChecker& VampR2000icPlanner::getCollisionChecker() {
    return pimpl_->checker_;
}

const CollisionChecker& VampR2000icPlanner::getCollisionChecker() const {
    return pimpl_->checker_;
}

bool VampR2000icPlanner::planFreespace(const std::vector<double>& start_joints,
                                      const std::vector<double>& target_joints,
                                      std::vector<std::vector<double>>& trajectory_out,
                                      double planning_time,
                                      double range,
                                      double safety_margin,
                                      const std::string& planner_type) {
    if (!pimpl_->initialized_ || start_joints.size() < 6 || target_joints.size() < 6) {
        return false;
    }

    // Check endpoints first
    if (checkCollision(start_joints, safety_margin) || checkCollision(target_joints, safety_margin)) {
        std::cerr << "[VampPlanner] Start or Target configuration is in collision!" << std::endl;
        return false;
    }

    ompl::geometric::SimpleSetup ss(pimpl_->space_);

    // Set validity checker & motion validator with current obstacle state
    auto checker = std::make_shared<VampStateValidityChecker>(ss.getSpaceInformation(), pimpl_->checker_, static_cast<float>(safety_margin));
    auto validator = std::make_shared<VampMotionValidator>(ss.getSpaceInformation(), pimpl_->checker_, static_cast<float>(safety_margin), range);
    ss.setStateValidityChecker(checker);
    ss.getSpaceInformation()->setMotionValidator(validator);

    // Set planner
    if (planner_type == "RRTstar" || planner_type == "RRT*") {
        auto rrtstar = std::make_shared<ompl::geometric::RRTstar>(ss.getSpaceInformation());
        rrtstar->setRange(range);
        ss.setPlanner(rrtstar);
    } else if (planner_type == "PRM") {
        auto prm = std::make_shared<ompl::geometric::PRM>(ss.getSpaceInformation());
        ss.setPlanner(prm);
    } else {
        auto rrtconn = std::make_shared<ompl::geometric::RRTConnect>(ss.getSpaceInformation());
        rrtconn->setRange(range);
        ss.setPlanner(rrtconn);
    }

    // Set start and goal
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start(pimpl_->space_);
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal(pimpl_->space_);
    for (int i = 0; i < 6; ++i) {
        start->values[i] = start_joints[i];
        goal->values[i] = target_joints[i];
    }
    ss.setStartAndGoalStates(start, goal);

    // Solve
    ompl::base::PlannerStatus solved = ss.solve(planning_time);
    if (solved && ss.haveExactSolutionPath()) {
        auto& path = ss.getSolutionPath();

        // OMPL Post-processing: Shortcut pruning + B-Spline high-order smoothing
        ompl::geometric::PathSimplifier simplifier(ss.getSpaceInformation());
        simplifier.simplifyMax(path);
        simplifier.smoothBSpline(path, 3);
        path.interpolate(); // Uniformly interpolate waypoints

        trajectory_out.clear();
        trajectory_out.reserve(path.getStateCount());

        for (size_t i = 0; i < path.getStateCount(); ++i) {
            const auto* rstate = path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
            std::vector<double> wp(6);
            for (int j = 0; j < 6; ++j) {
                wp[j] = rstate->values[j];
            }
            trajectory_out.push_back(wp);
        }
        return true;
    }

    return false;
}

} // namespace vamp_r2000ic
