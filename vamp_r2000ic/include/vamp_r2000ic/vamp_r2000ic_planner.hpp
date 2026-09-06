#pragma once

#include "r2000ic_collision.hpp"
#include "vamp_toppra.hpp"
#include <vector>
#include <string>
#include <memory>

#if defined(_WIN32)
  #if defined(VAMP_R2000IC_EXPORTS)
    #define VAMP_EXPORT __declspec(dllexport)
  #else
    #define VAMP_EXPORT __declspec(dllimport)
  #endif
#else
  #define VAMP_EXPORT __attribute__((visibility("default")))
#endif

namespace vamp_r2000ic {

class VAMP_EXPORT VampR2000icPlanner {
public:
    VampR2000icPlanner();
    ~VampR2000icPlanner();

    /**
     * @brief Initialize OMPL state space and setup for Fanuc R-2000iC/165F.
     */
    bool init();

    /**
     * @brief Configure the 6 joint origins and axes from URDF.
     */
    void setJointOrigins(const double origins_xyz[18], const double origins_rpy[18], const double axes_xyz[18]);

    /**
     * @brief Configure joint velocity and acceleration limits for TOPP-RA time parameterization.
     */
    void setLimits(const double vel_limits[6], const double acc_limits[6]);

    /**
     * @brief Load SRDF file to configure Allowed Collision Matrix (ACM).
     * @param srdf_path Path to SRDF XML file
     * @return true on success, false on failure
     */
    bool loadSRDF(const std::string& srdf_path);

    /**
     * @brief Modify Allowed Collision Matrix whitelist dynamically.
     * @param link1 Robot link or obstacle name
     * @param link2 Robot link or obstacle name
     * @param allowed true to exempt from collision, false to restore
     * @return true on success
     */
    bool setAllowedCollision(const std::string& link1, const std::string& link2, bool allowed);

    /**
     * @brief Add an Axis-Aligned Bounding Box obstacle into the collision environment.
     * @param name Unique identifier
     * @param cx, cy, cz Box center position in world coords
     * @param dx, dy, dz Box dimensions (length, width, height)
     */
    void addObstacleBox(const std::string& name, double cx, double cy, double cz,
                         double dx, double dy, double dz);

    /**
     * @brief Add multiple Axis-Aligned Bounding Box obstacles (e.g. from Octree voxels) into the collision environment.
     * @param name Unique identifier for this obstacle group
     * @param boxes Array of AABB bounding boxes
     */
    void addObstacleBoxes(const std::string& name, const std::vector<AABB>& boxes);

    /**
     * @brief Remove an obstacle by name.
     */
    bool removeObstacle(const std::string& name);

    /**
     * @brief Clear all obstacles in the scene.
     */
    void clearObstacles();

    /**
     * @brief Check whether given 6 joint angles collide with environment or self.
     * @return true if collision detected, false if safe.
     */
    bool checkCollision(const std::vector<double>& joints, double safety_margin = 0.025);

    /**
     * @brief High-speed motion planning with VAMP SIMD validation and OMPL.
     * @param start_joints 6 starting joint angles
     * @param target_joints 6 goal joint angles
     * @param trajectory_out Output joint trajectory [N x 6]
     * @param planning_time Maximum allowed time in seconds (default 5.0)
     * @param range Step size / interpolation range (default 0.02)
     * @param safety_margin Extra safety clearance distance in meters (default 0.025)
     * @param planner_type "PRM" / "RRTConnect" / "RRTstar" (default "PRM")
     * @return true on success, false on failure
     */
    bool planFreespace(const std::vector<double>& start_joints,
                       const std::vector<double>& target_joints,
                       std::vector<std::vector<double>>& trajectory_out,
                       double planning_time = 5.0,
                       double range = 0.02,
                       double safety_margin = 0.025,
                       const std::string& planner_type = "PRM");

    /**
     * @brief cuRobo-style full planning pipeline: OMPL (PRM) -> B-Spline + L-BFGS -> TOPP-RA.
     * Generates a fully-dynamic, time-optimal, collision-free trajectory in milliseconds without TrajOpt.
     */
    bool planTrajectory(const std::vector<double>& start_joints,
                        const std::vector<double>& target_joints,
                        TimedTrajectory& trajectory_out,
                        double max_velocity_scaling = 1.0,
                        double max_acceleration_scaling = 1.0,
                        double planning_time = 5.0,
                        double range = 0.02,
                        double safety_margin = 0.025,
                        const std::string& planner_type = "PRM");

    /**
     * @brief Precompute/warm up the persistent PRM roadmap.
     * @param warmup_time CPU time budget for growing roadmap in seconds (default 0.3s)
     */
    void warmupRoadmap(double warmup_time = 0.3);

    /**
     * @brief Attach workpiece collision spheres to a robot link for attached-body collision checking.
     * @param name Unique obstacle/workpiece name
     * @param link_index Robot link index (e.g. 6 for tool0/flange)
     * @param spheres Bounding spheres in the link's local coordinate frame
     */
    void addAttachedSpheres(const std::string& name, int link_index, const std::vector<Sphere>& spheres);

    /**
     * @brief Remove attached workpiece spheres when detaching.
     */
    bool removeAttachedSpheres(const std::string& name);

    /**
     * @brief Clear all attached workpiece spheres.
     */
    void clearAttachedSpheres();

    /**
     * @brief Get direct reference to the collision checker.
     */
    CollisionChecker& getCollisionChecker();
    const CollisionChecker& getCollisionChecker() const;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace vamp_r2000ic
