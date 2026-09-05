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

#ifdef __cplusplus
}
#endif
