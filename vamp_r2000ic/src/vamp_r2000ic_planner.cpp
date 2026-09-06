#include "vamp_r2000ic/vamp_r2000ic_planner.hpp"
#include "vamp_r2000ic/vamp_ompl_adapter.hpp"
#include "vamp_r2000ic/vamp_bspline_optimizer.hpp"

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/base/PlannerTerminationCondition.h>

namespace vamp_r2000ic {

struct VampR2000icPlanner::Impl {
    CollisionChecker checker_;
    std::shared_ptr<ompl::base::RealVectorStateSpace> space_;
    ompl::base::SpaceInformationPtr si_;
    std::shared_ptr<VampStateValidityChecker> checker_adapter_;
    std::shared_ptr<VampMotionValidator> validator_adapter_;
    std::shared_ptr<ompl::geometric::PRM> persistent_prm_;
    std::shared_ptr<ompl::base::ProblemDefinition> pdef_;

    bool initialized_ = false;
    bool roadmap_warmed_ = false;
    float current_safety_margin_ = 0.025f;
    double current_range_ = 0.02;
    std::vector<double> vel_limits_{ 2.2689, 2.0071, 2.1817, 3.8397, 3.8397, 5.5851 };
    std::vector<double> acc_limits_{ 6.0, 5.0, 6.0, 10.0, 10.0, 15.0 };
};

VampR2000icPlanner::VampR2000icPlanner() : pimpl_(std::make_unique<Impl>()) {}
VampR2000icPlanner::~VampR2000icPlanner() = default;

void VampR2000icPlanner::setLimits(const double vel_limits[6], const double acc_limits[6]) {
    if (vel_limits) pimpl_->vel_limits_.assign(vel_limits, vel_limits + 6);
    if (acc_limits) pimpl_->acc_limits_.assign(acc_limits, acc_limits + 6);
}

bool VampR2000icPlanner::init() {
    try {
        pimpl_->space_ = createR2000icStateSpace();
        pimpl_->si_ = std::make_shared<ompl::base::SpaceInformation>(pimpl_->space_);
        pimpl_->checker_adapter_ = std::make_shared<VampStateValidityChecker>(pimpl_->si_, pimpl_->checker_, pimpl_->current_safety_margin_);
        pimpl_->validator_adapter_ = std::make_shared<VampMotionValidator>(pimpl_->si_, pimpl_->checker_, pimpl_->current_safety_margin_, pimpl_->current_range_);

        pimpl_->si_->setStateValidityChecker(pimpl_->checker_adapter_);
        pimpl_->si_->setMotionValidator(pimpl_->validator_adapter_);
        pimpl_->si_->setup();

        // Initialize persistent PRM planner
        pimpl_->persistent_prm_ = std::make_shared<ompl::geometric::PRM>(pimpl_->si_);
        pimpl_->pdef_ = std::make_shared<ompl::base::ProblemDefinition>(pimpl_->si_);
        pimpl_->persistent_prm_->setProblemDefinition(pimpl_->pdef_);
        pimpl_->persistent_prm_->setup();

        pimpl_->initialized_ = true;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[VampPlanner] init exception: " << e.what() << std::endl;
        return false;
    } catch (...) {
        std::cerr << "[VampPlanner] init unknown exception" << std::endl;
        return false;
    }
}


void VampR2000icPlanner::setJointOrigins(const double origins_xyz[18], const double origins_rpy[18], const double axes_xyz[18]) {
    pimpl_->checker_.setJointOrigins(origins_xyz, origins_rpy, axes_xyz);
}

bool VampR2000icPlanner::loadSRDF(const std::string& srdf_path) {
    bool res = pimpl_->checker_.loadSRDF(srdf_path);
    if (res && pimpl_->persistent_prm_) {
        pimpl_->persistent_prm_->clearQuery();
    }
    return res;
}

bool VampR2000icPlanner::setAllowedCollision(const std::string& link1, const std::string& link2, bool allowed) {
    bool res = pimpl_->checker_.setAllowedCollision(link1, link2, allowed);
    if (res && pimpl_->persistent_prm_) {
        pimpl_->persistent_prm_->clearQuery();
    }
    return res;
}

void VampR2000icPlanner::warmupRoadmap(double warmup_time) {
    if (!pimpl_->initialized_ || !pimpl_->persistent_prm_) return;
    pimpl_->persistent_prm_->growRoadmap(warmup_time);
    pimpl_->roadmap_warmed_ = true;
}

void VampR2000icPlanner::addObstacleBox(const std::string& name, double cx, double cy, double cz,
                                       double dx, double dy, double dz) {
    pimpl_->checker_.addBoxCenterDim(name, static_cast<float>(cx), static_cast<float>(cy), static_cast<float>(cz),
                                     static_cast<float>(dx), static_cast<float>(dy), static_cast<float>(dz));
    if (pimpl_->persistent_prm_) {
        pimpl_->persistent_prm_->clearQuery();
    }
}

void VampR2000icPlanner::addObstacleBoxes(const std::string& name, const std::vector<AABB>& boxes) {
    pimpl_->checker_.addBoxes(name, boxes);
    if (pimpl_->persistent_prm_) {
        pimpl_->persistent_prm_->clearQuery();
    }
}

bool VampR2000icPlanner::removeObstacle(const std::string& name) {
    bool res = pimpl_->checker_.removeObstacle(name);
    if (res && pimpl_->persistent_prm_) {
        pimpl_->persistent_prm_->clearQuery();
    }
    return res;
}

void VampR2000icPlanner::clearObstacles() {
    pimpl_->checker_.clearObstacles();
    if (pimpl_->persistent_prm_) {
        pimpl_->persistent_prm_->clearQuery();
    }
}

void VampR2000icPlanner::addAttachedSpheres(const std::string& name, int link_index, const std::vector<Sphere>& spheres) {
    pimpl_->checker_.addAttachedSpheres(name, link_index, spheres);
    if (pimpl_->persistent_prm_) {
        pimpl_->persistent_prm_->clearQuery();
    }
}

bool VampR2000icPlanner::removeAttachedSpheres(const std::string& name) {
    bool res = pimpl_->checker_.removeAttachedSpheres(name);
    if (res && pimpl_->persistent_prm_) {
        pimpl_->persistent_prm_->clearQuery();
    }
    return res;
}

void VampR2000icPlanner::clearAttachedSpheres() {
    pimpl_->checker_.clearAttachedSpheres();
    if (pimpl_->persistent_prm_) {
        pimpl_->persistent_prm_->clearQuery();
    }
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

    // Check endpoints first with SIMD kernel
    if (checkCollision(start_joints, safety_margin) || checkCollision(target_joints, safety_margin)) {
        std::cerr << "[VampPlanner] Start or Target configuration is in collision!" << std::endl;
        return false;
    }

    // Fast path: cuRobo-style PRMGraphPlanner with persistent roadmap & warmup
    if (planner_type == "PRM" || planner_type == "prm" || planner_type.empty()) {
        if (!pimpl_->roadmap_warmed_) {
            pimpl_->persistent_prm_->growRoadmap(0.2);
            pimpl_->roadmap_warmed_ = true;
        }

        pimpl_->persistent_prm_->clearQuery();
        pimpl_->pdef_->clearStartStates();
        pimpl_->pdef_->clearGoal();
        pimpl_->pdef_->clearSolutionPaths();

        ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start(pimpl_->space_);
        ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal(pimpl_->space_);
        for (int i = 0; i < 6; ++i) {
            start->values[i] = start_joints[i];
            goal->values[i] = target_joints[i];
        }
        pimpl_->pdef_->setStartAndGoalStates(start, goal);

        ompl::base::PlannerStatus solved = pimpl_->persistent_prm_->solve(ompl::base::timedPlannerTerminationCondition(planning_time));
        if (solved && pimpl_->pdef_->hasExactSolution()) {
            auto path_ptr = pimpl_->pdef_->getSolutionPath();
            auto* geo_path = dynamic_cast<ompl::geometric::PathGeometric*>(path_ptr.get());
            if (geo_path) {
                ompl::geometric::PathSimplifier simplifier(pimpl_->si_);
                simplifier.simplifyMax(*geo_path);
                simplifier.smoothBSpline(*geo_path, 3);
                geo_path->interpolate();

                trajectory_out.clear();
                trajectory_out.reserve(geo_path->getStateCount());
                for (size_t i = 0; i < geo_path->getStateCount(); ++i) {
                    const auto* rstate = geo_path->getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
                    std::vector<double> wp(6);
                    for (int j = 0; j < 6; ++j) wp[j] = rstate->values[j];
                    trajectory_out.push_back(wp);
                }
                return true;
            }
        }
        return false;
    }

    // Fallback: single-query tree search (RRTConnect / RRTstar)
    ompl::geometric::SimpleSetup ss(pimpl_->space_);
    auto checker = std::make_shared<VampStateValidityChecker>(ss.getSpaceInformation(), pimpl_->checker_, static_cast<float>(safety_margin));
    auto validator = std::make_shared<VampMotionValidator>(ss.getSpaceInformation(), pimpl_->checker_, static_cast<float>(safety_margin), range);
    ss.setStateValidityChecker(checker);
    ss.getSpaceInformation()->setMotionValidator(validator);

    if (planner_type == "RRTstar" || planner_type == "RRT*") {
        auto rrtstar = std::make_shared<ompl::geometric::RRTstar>(ss.getSpaceInformation());
        rrtstar->setRange(range);
        ss.setPlanner(rrtstar);
    } else {
        auto rrtconn = std::make_shared<ompl::geometric::RRTConnect>(ss.getSpaceInformation());
        rrtconn->setRange(range);
        ss.setPlanner(rrtconn);
    }

    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start(pimpl_->space_);
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal(pimpl_->space_);
    for (int i = 0; i < 6; ++i) {
        start->values[i] = start_joints[i];
        goal->values[i] = target_joints[i];
    }
    ss.setStartAndGoalStates(start, goal);

    ompl::base::PlannerStatus solved = ss.solve(planning_time);
    if (solved && ss.haveExactSolutionPath()) {
        auto& path = ss.getSolutionPath();
        ompl::geometric::PathSimplifier simplifier(ss.getSpaceInformation());
        simplifier.simplifyMax(path);
        simplifier.smoothBSpline(path, 3);
        path.interpolate();

        trajectory_out.clear();
        trajectory_out.reserve(path.getStateCount());
        for (size_t i = 0; i < path.getStateCount(); ++i) {
            const auto* rstate = path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
            std::vector<double> wp(6);
            for (int j = 0; j < 6; ++j) wp[j] = rstate->values[j];
            trajectory_out.push_back(wp);
        }
        return true;
    }

    return false;
}

bool VampR2000icPlanner::planTrajectory(const std::vector<double>& start_joints,
                                        const std::vector<double>& target_joints,
                                        TimedTrajectory& trajectory_out,
                                        double max_velocity_scaling,
                                        double max_acceleration_scaling,
                                        double planning_time,
                                        double range,
                                        double safety_margin,
                                        const std::string& planner_type) {
    trajectory_out.clear();
    if (!pimpl_->initialized_ || start_joints.size() < 6 || target_joints.size() < 6) {
        return false;
    }

    // Step 1: OMPL (PRM/RRT) global search
    std::vector<std::vector<double>> seed_waypoints;
    bool ok = planFreespace(start_joints, target_joints, seed_waypoints, planning_time, range, safety_margin, planner_type);
    if (!ok || seed_waypoints.empty()) {
        return false;
    }

    // Step 2: B-Spline + L-BFGS optimization (cuRobo style)
    int num_cp = 12;
    int num_samples = 40;
    VampBSplineOptimizer optimizer(pimpl_->checker_, num_cp, num_samples);
    optimizer.setWeights(1.0, 100.0, 200.0, safety_margin);

    Eigen::MatrixXd control_points;
    optimizer.getBSpline().initFromWaypoints(seed_waypoints, start_joints, target_joints, control_points);
    optimizer.optimize(control_points, 25);

    // Evaluate dense samples from optimized B-Spline
    Eigen::MatrixXd optimized_samples;
    optimizer.getBSpline().evaluate(control_points, optimized_samples);

    int rows = static_cast<int>(optimized_samples.rows());
    std::vector<std::vector<double>> opt_waypoints(rows, std::vector<double>(6));
    for (int i = 0; i < rows; ++i) {
        for (int d = 0; d < 6; ++d) {
            opt_waypoints[i][d] = optimized_samples(i, d);
        }
    }

    return parameterize(opt_waypoints, trajectory_out, max_velocity_scaling, max_acceleration_scaling);
}

bool VampR2000icPlanner::parameterize(const std::vector<std::vector<double>>& waypoints,
                                      TimedTrajectory& trajectory_out,
                                      double max_velocity_scaling,
                                      double max_acceleration_scaling,
                                      double sample_dt) {
    trajectory_out.clear();
    if (waypoints.size() < 2) {
        return false;
    }

    VampToppra toppra_opt;
    toppra_opt.setLimits(pimpl_->vel_limits_, pimpl_->acc_limits_);
    if (toppra_opt.parameterize(waypoints, trajectory_out, max_velocity_scaling, max_acceleration_scaling, sample_dt)
        && !trajectory_out.empty()) {
        return true;
    }

    // Fallback: uniform timestamps if TOPP-RA cannot solve (e.g. fewer than 3 waypoints)
    const double dt = 0.02;
    trajectory_out.positions = waypoints;
    trajectory_out.time_stamps.resize(waypoints.size());
    for (size_t i = 0; i < waypoints.size(); ++i) {
        trajectory_out.time_stamps[i] = static_cast<double>(i) * dt;
    }
    return true;
}

} // namespace vamp_r2000ic
