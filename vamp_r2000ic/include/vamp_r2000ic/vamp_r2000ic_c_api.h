#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32)
  #if defined(VAMP_R2000IC_EXPORTS)
    #define VAMP_EXPORT __declspec(dllexport)
  #else
    #define VAMP_EXPORT __declspec(dllimport)
  #endif
#else
  #define VAMP_EXPORT __attribute__((visibility("default")))
#endif

/**
 * @brief Opaque handle for VampR2000icPlanner instance.
 */
typedef void* VampPlannerHandle;

/**
 * @brief Creates a new VampR2000icPlanner instance.
 * @return Handle to the planner instance, or NULL on failure.
 */
VAMP_EXPORT VampPlannerHandle vamp_r2000ic_create(void);

/**
 * @brief Destroys the VampR2000icPlanner instance.
 * @param handle Planner handle.
 */
VAMP_EXPORT void vamp_r2000ic_destroy(VampPlannerHandle handle);

/**
 * @brief Initializes the planner (state space, SIMD forward kinematics, collision checker).
 * @param handle Planner handle.
 * @return 1 on success, 0 on failure.
 */
VAMP_EXPORT int vamp_r2000ic_init(VampPlannerHandle handle);

/**
 * @brief Dynamically configures the 6 joint origins (xyz, rpy) and axes from URDF.
 * @param handle Planner handle.
 * @param origins_xyz Array of 18 doubles (6 * [x, y, z])
 * @param origins_rpy Array of 18 doubles (6 * [r, p, y])
 * @param axes_xyz Array of 18 doubles (6 * [axis_x, axis_y, axis_z])
 * @return 1 on success, 0 on failure.
 */
VAMP_EXPORT int vamp_r2000ic_set_joint_origins(VampPlannerHandle handle,
                                               const double* origins_xyz,
                                               const double* origins_rpy,
                                               const double* axes_xyz);

/**
 * @brief Configures 6-axis velocity and acceleration limits for TOPP-RA time optimal parameterization.
 * @param handle Planner handle.
 * @param vel_limits Array of 6 doubles (joint velocity limits in rad/s)
 * @param acc_limits Array of 6 doubles (joint acceleration limits in rad/s^2)
 * @return 1 on success, 0 on failure.
 */
VAMP_EXPORT int vamp_r2000ic_set_limits(VampPlannerHandle handle,
                                        const double* vel_limits,
                                        const double* acc_limits);

/**
 * @brief Parses an SRDF XML file to configure Allowed Collision Matrix (ACM) rules.
 * @param handle Planner handle.
 * @param srdf_path Path to SRDF XML file.
 * @return 1 on success, 0 on failure.
 */
VAMP_EXPORT int vamp_r2000ic_load_srdf(VampPlannerHandle handle, const char* srdf_path);

/**
 * @brief Modifies the Allowed Collision Matrix (ACM) whitelist for two links or obstacles.
 * @param handle Planner handle.
 * @param link1 First link or obstacle name.
 * @param link2 Second link or obstacle name.
 * @param allowed 1 to exempt from collision, 0 to restore collision testing.
 * @return 1 on success, 0 on failure.
 */
VAMP_EXPORT int vamp_r2000ic_set_allowed_collision(VampPlannerHandle handle,
                                                   const char* link1,
                                                   const char* link2,
                                                   int allowed);

/**
 * @brief Adds an axis-aligned box obstacle.
 * @param handle Planner handle.
 * @param name Obstacle name.
 * @param x, y, z Center coordinate.
 * @param dim_x, dim_y, dim_z Extents along X, Y, Z.
 * @return 1 on success, 0 on failure.
 */
VAMP_EXPORT int vamp_r2000ic_add_box(VampPlannerHandle handle,
                                     const char* name,
                                     double x, double y, double z,
                                     double dim_x, double dim_y, double dim_z);

/**
 * @brief Adds multiple axis-aligned box obstacles (e.g. from an Octree) into the collision environment.
 * @param handle Planner handle.
 * @param name Obstacle group name.
 * @param aabb_min_max_array Flat array of size (box_count * 6): [min_x, min_y, min_z, max_x, max_y, max_z, ...]
 * @param box_count Number of AABB boxes.
 * @return 1 on success, 0 on failure.
 */
VAMP_EXPORT int vamp_r2000ic_add_boxes(VampPlannerHandle handle,
                                       const char* name,
                                       const double* aabb_min_max_array,
                                       int box_count);

/**
 * @brief Removes an obstacle from the collision environment.
 * @param handle Planner handle.
 * @param name Obstacle name.
 * @return 1 on success, 0 on failure.
 */
VAMP_EXPORT int vamp_r2000ic_remove_obstacle(VampPlannerHandle handle, const char* name);

/**
 * @brief Checks collision for the given 6-DOF joint angles using AVX2 SIMD.
 * @param handle Planner handle.
 * @param joints_6 Array of 6 joint values in radians.
 * @return 1 if collision occurs, 0 if safe/valid.
 */
VAMP_EXPORT int vamp_r2000ic_check_collision(VampPlannerHandle handle, const double* joints_6);

/**
 * @brief Plans a collision-free path in joint space.
 * @param handle Planner handle.
 * @param start_joints_6 6-element start joint angles.
 * @param goal_joints_6 6-element goal joint angles.
 * @param out_trajectory Output flat array of size (max_points * 6) to receive waypoints.
 * @param out_num_points Output pointer to receive the actual number of waypoints.
 * @param max_points Maximum number of waypoints that out_trajectory can hold.
 * @param timeout Maximum planning timeout in seconds.
 * @param step_size OMPL step range in radians.
 * @param margin Safety distance margin in meters.
 * @param planner_type "RRTConnect", "RRTstar", or "PRM".
 * @return 1 on success, 0 on failure.
 */
VAMP_EXPORT int vamp_r2000ic_plan_freespace(VampPlannerHandle handle,
                                           const double* start_joints_6,
                                           const double* goal_joints_6,
                                           double* out_trajectory,
                                           int* out_num_points,
                                           int max_points,
                                           double timeout,
                                           double step_size,
                                           double margin,
                                           const char* planner_type);

/**
 * @brief Plans a cuRobo-style fully parameterized trajectory (positions, velocities, accelerations, time_stamps)
 * using PRM -> B-Spline L-BFGS -> TOPP-RA, bypassing TrajOpt.
 */
VAMP_EXPORT int vamp_r2000ic_plan_trajectory(VampPlannerHandle handle,
                                             const double* start_joints_6,
                                             const double* goal_joints_6,
                                             double* out_positions,
                                             double* out_velocities,
                                             double* out_accelerations,
                                             double* out_time_stamps,
                                             int* out_num_points,
                                             int max_points,
                                             double max_vel_scaling,
                                             double max_acc_scaling,
                                             double timeout,
                                             double step_size,
                                             double margin,
                                             const char* planner_type);

/**
 * @brief Precomputes and warms up the persistent PRM roadmap graph.
 * @param handle Planner handle.
 * @param warmup_time Time in seconds to grow roadmap.
 * @return 1 on success, 0 on failure.
 */
VAMP_EXPORT int vamp_r2000ic_warmup_roadmap(VampPlannerHandle handle, double warmup_time);

/**
 * @brief Attaches workpiece bounding spheres to a robot link for attached-body SIMD collision checks.
 * @param handle Planner handle.
 * @param name Workpiece/obstacle name.
 * @param link_index Robot link index (6 for tool0/J6_link).
 * @param spheres_xyzr Flat array of [x, y, z, radius] in link local coordinates.
 * @param sphere_count Number of spheres.
 * @return 1 on success, 0 on failure.
 */
VAMP_EXPORT int vamp_r2000ic_add_attached_spheres(VampPlannerHandle handle,
                                                  const char* name,
                                                  int link_index,
                                                  const double* spheres_xyzr,
                                                  int sphere_count);

/**
 * @brief Removes attached workpiece spheres when detaching.
 * @param handle Planner handle.
 * @param name Workpiece/obstacle name.
 * @return 1 on success, 0 on failure.
 */
VAMP_EXPORT int vamp_r2000ic_remove_attached_spheres(VampPlannerHandle handle, const char* name);

/**
 * @brief Clears all attached workpiece spheres.
 * @param handle Planner handle.
 * @return 1 on success, 0 on failure.
 */
VAMP_EXPORT int vamp_r2000ic_clear_attached_spheres(VampPlannerHandle handle);

#ifdef __cplusplus
}
#endif
