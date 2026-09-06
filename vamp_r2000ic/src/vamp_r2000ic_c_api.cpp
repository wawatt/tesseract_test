#include "vamp_r2000ic/vamp_r2000ic_c_api.h"
#include "vamp_r2000ic/vamp_r2000ic_planner.hpp"
#include <vector>
#include <string>
#include <cstring>
#include <algorithm>

VampPlannerHandle vamp_r2000ic_create(void) {
    try {
        return new vamp_r2000ic::VampR2000icPlanner();
    } catch (...) {
        return nullptr;
    }
}

void vamp_r2000ic_destroy(VampPlannerHandle handle) {
    if (handle) {
        delete static_cast<vamp_r2000ic::VampR2000icPlanner*>(handle);
    }
}

int vamp_r2000ic_init(VampPlannerHandle handle) {
    if (!handle) return 0;
    auto* planner = static_cast<vamp_r2000ic::VampR2000icPlanner*>(handle);
    try {
        return planner->init() ? 1 : 0;
    } catch (...) {
        return 0;
    }
}

int vamp_r2000ic_set_joint_origins(VampPlannerHandle handle,
                                   const double* origins_xyz,
                                   const double* origins_rpy,
                                   const double* axes_xyz) {
    if (!handle || !origins_xyz || !origins_rpy || !axes_xyz) return 0;
    auto* planner = static_cast<vamp_r2000ic::VampR2000icPlanner*>(handle);
    try {
        planner->setJointOrigins(origins_xyz, origins_rpy, axes_xyz);
        return 1;
    } catch (...) {
        return 0;
    }
}

int vamp_r2000ic_set_limits(VampPlannerHandle handle,
                            const double* vel_limits,
                            const double* acc_limits) {
    if (!handle || !vel_limits || !acc_limits) return 0;
    auto* planner = static_cast<vamp_r2000ic::VampR2000icPlanner*>(handle);
    try {
        planner->setLimits(vel_limits, acc_limits);
        return 1;
    } catch (...) {
        return 0;
    }
}

int vamp_r2000ic_load_srdf(VampPlannerHandle handle, const char* srdf_path) {
    if (!handle || !srdf_path) return 0;
    auto* planner = static_cast<vamp_r2000ic::VampR2000icPlanner*>(handle);
    try {
        return planner->loadSRDF(std::string(srdf_path)) ? 1 : 0;
    } catch (...) {
        return 0;
    }
}

int vamp_r2000ic_set_allowed_collision(VampPlannerHandle handle,
                                       const char* link1,
                                       const char* link2,
                                       int allowed) {
    if (!handle || !link1 || !link2) return 0;
    auto* planner = static_cast<vamp_r2000ic::VampR2000icPlanner*>(handle);
    try {
        return planner->setAllowedCollision(std::string(link1), std::string(link2), allowed != 0) ? 1 : 0;
    } catch (...) {
        return 0;
    }
}

int vamp_r2000ic_is_collision_allowed(VampPlannerHandle handle,
                                      const char* link1,
                                      const char* link2) {
    if (!handle || !link1 || !link2) return 0;
    auto* planner = static_cast<vamp_r2000ic::VampR2000icPlanner*>(handle);
    try {
        return planner->isCollisionAllowed(std::string(link1), std::string(link2)) ? 1 : 0;
    } catch (...) {
        return 0;
    }
}

int vamp_r2000ic_check_collision_detailed(VampPlannerHandle handle,
                                          const double* joints_6,
                                          VampContactInfo* out_contacts,
                                          int max_contacts,
                                          int* out_count,
                                          double contact_distance) {
    if (out_count) *out_count = 0;
    if (!handle || !joints_6 || !out_contacts || !out_count || max_contacts <= 0) return 0;
    auto* planner = static_cast<vamp_r2000ic::VampR2000icPlanner*>(handle);
    try {
        std::vector<double> q(joints_6, joints_6 + 6);
        std::vector<vamp_r2000ic::ContactHit> hits;
        if (!planner->checkCollisionDetailed(q, hits, contact_distance) || hits.empty()) {
            return 0;
        }
        int count = std::min(static_cast<int>(hits.size()), max_contacts);
        for (int i = 0; i < count; ++i) {
            const auto& h = hits[static_cast<size_t>(i)];
            std::strncpy(out_contacts[i].link1, h.link1.c_str(), sizeof(out_contacts[i].link1) - 1);
            out_contacts[i].link1[sizeof(out_contacts[i].link1) - 1] = '\0';
            std::strncpy(out_contacts[i].link2, h.link2.c_str(), sizeof(out_contacts[i].link2) - 1);
            out_contacts[i].link2[sizeof(out_contacts[i].link2) - 1] = '\0';
            out_contacts[i].distance = h.distance;
            for (int k = 0; k < 3; ++k) {
                out_contacts[i].point1[k] = h.point1[k];
                out_contacts[i].point2[k] = h.point2[k];
                out_contacts[i].normal[k] = h.normal[k];
            }
        }
        *out_count = count;
        return 1;
    } catch (...) {
        return 0;
    }
}

int vamp_r2000ic_add_box(VampPlannerHandle handle,
                         const char* name,
                         double x, double y, double z,
                         double dim_x, double dim_y, double dim_z) {
    if (!handle || !name) return 0;
    auto* planner = static_cast<vamp_r2000ic::VampR2000icPlanner*>(handle);
    try {
        planner->addObstacleBox(std::string(name), x, y, z, dim_x, dim_y, dim_z);
        return 1;
    } catch (...) {
        return 0;
    }
}

int vamp_r2000ic_add_boxes(VampPlannerHandle handle,
                           const char* name,
                           const double* aabb_min_max_array,
                           int box_count) {
    if (!handle || !name || !aabb_min_max_array || box_count <= 0) return 0;
    auto* planner = static_cast<vamp_r2000ic::VampR2000icPlanner*>(handle);
    try {
        std::vector<vamp_r2000ic::AABB> boxes;
        boxes.reserve(box_count);
        for (int i = 0; i < box_count; ++i) {
            int base_idx = i * 6;
            boxes.emplace_back(
                vamp_r2000ic::Point3(static_cast<float>(aabb_min_max_array[base_idx + 0]),
                                     static_cast<float>(aabb_min_max_array[base_idx + 1]),
                                     static_cast<float>(aabb_min_max_array[base_idx + 2])),
                vamp_r2000ic::Point3(static_cast<float>(aabb_min_max_array[base_idx + 3]),
                                     static_cast<float>(aabb_min_max_array[base_idx + 4]),
                                     static_cast<float>(aabb_min_max_array[base_idx + 5]))
            );
        }
        planner->addObstacleBoxes(std::string(name), boxes);
        return 1;
    } catch (...) {
        return 0;
    }
}

int vamp_r2000ic_remove_obstacle(VampPlannerHandle handle, const char* name) {
    if (!handle || !name) return 0;
    auto* planner = static_cast<vamp_r2000ic::VampR2000icPlanner*>(handle);
    try {
        return planner->removeObstacle(std::string(name)) ? 1 : 0;
    } catch (...) {
        return 0;
    }
}

int vamp_r2000ic_check_collision(VampPlannerHandle handle, const double* joints_6) {
    if (!handle || !joints_6) return 1; // Return 1 (collision) on invalid handle/input
    auto* planner = static_cast<vamp_r2000ic::VampR2000icPlanner*>(handle);
    try {
        std::vector<double> q(joints_6, joints_6 + 6);
        return planner->checkCollision(q) ? 1 : 0;
    } catch (...) {
        return 1;
    }
}

int vamp_r2000ic_plan_freespace(VampPlannerHandle handle,
                               const double* start_joints_6,
                               const double* goal_joints_6,
                               double* out_trajectory,
                               int* out_num_points,
                               int max_points,
                               double timeout,
                               double step_size,
                               double margin,
                               const char* planner_type) {
    if (!handle || !start_joints_6 || !goal_joints_6 || !out_trajectory || !out_num_points || max_points <= 0) {
        return 0;
    }
    auto* planner = static_cast<vamp_r2000ic::VampR2000icPlanner*>(handle);
    try {
        std::vector<double> start_q(start_joints_6, start_joints_6 + 6);
        std::vector<double> goal_q(goal_joints_6, goal_joints_6 + 6);
        std::vector<std::vector<double>> trajectory;
        std::string ptype = planner_type ? std::string(planner_type) : "RRTConnect";

        bool ok = planner->planFreespace(start_q, goal_q, trajectory, timeout, step_size, margin, ptype);
        if (!ok || trajectory.empty()) {
            *out_num_points = 0;
            return 0;
        }

        int count = std::min(static_cast<int>(trajectory.size()), max_points);
        for (int i = 0; i < count; ++i) {
            for (int j = 0; j < 6; ++j) {
                out_trajectory[i * 6 + j] = trajectory[i][j];
            }
        }
        *out_num_points = count;
        return 1;
    } catch (...) {
        *out_num_points = 0;
        return 0;
    }
}

int vamp_r2000ic_plan_trajectory(VampPlannerHandle handle,
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
                                 const char* planner_type) {
    if (!handle || !start_joints_6 || !goal_joints_6 || !out_positions || !out_time_stamps || !out_num_points || max_points <= 0) {
        return 0;
    }
    auto* planner = static_cast<vamp_r2000ic::VampR2000icPlanner*>(handle);
    try {
        std::vector<double> start_q(start_joints_6, start_joints_6 + 6);
        std::vector<double> goal_q(goal_joints_6, goal_joints_6 + 6);
        vamp_r2000ic::TimedTrajectory traj;
        std::string ptype = planner_type ? std::string(planner_type) : "PRM";

        bool ok = planner->planTrajectory(start_q, goal_q, traj, max_vel_scaling, max_acc_scaling, timeout, step_size, margin, ptype);
        if (!ok || traj.empty()) {
            *out_num_points = 0;
            return 0;
        }

        int count = std::min(static_cast<int>(traj.positions.size()), max_points);
        for (int i = 0; i < count; ++i) {
            for (int j = 0; j < 6; ++j) {
                out_positions[i * 6 + j] = traj.positions[i][j];
                if (out_velocities && i < static_cast<int>(traj.velocities.size())) {
                    out_velocities[i * 6 + j] = traj.velocities[i][j];
                }
                if (out_accelerations && i < static_cast<int>(traj.accelerations.size())) {
                    out_accelerations[i * 6 + j] = traj.accelerations[i][j];
                }
            }
            out_time_stamps[i] = traj.time_stamps[i];
        }
        *out_num_points = count;
        return 1;
    } catch (...) {
        *out_num_points = 0;
        return 0;
    }
}

int vamp_r2000ic_parameterize(VampPlannerHandle handle,
                              const double* waypoints_flat,
                              int num_waypoints,
                              double* out_positions,
                              double* out_velocities,
                              double* out_accelerations,
                              double* out_time_stamps,
                              int* out_num_points,
                              int max_points,
                              double max_vel_scaling,
                              double max_acc_scaling,
                              double sample_dt) {
    if (!handle || !waypoints_flat || !out_positions || !out_time_stamps || !out_num_points
        || num_waypoints < 2 || max_points <= 0) {
        return 0;
    }
    auto* planner = static_cast<vamp_r2000ic::VampR2000icPlanner*>(handle);
    try {
        std::vector<std::vector<double>> waypoints(static_cast<size_t>(num_waypoints), std::vector<double>(6));
        for (int i = 0; i < num_waypoints; ++i) {
            for (int j = 0; j < 6; ++j) {
                waypoints[static_cast<size_t>(i)][static_cast<size_t>(j)] = waypoints_flat[i * 6 + j];
            }
        }

        vamp_r2000ic::TimedTrajectory traj;
        if (!planner->parameterize(waypoints, traj, max_vel_scaling, max_acc_scaling, sample_dt) || traj.empty()) {
            *out_num_points = 0;
            return 0;
        }

        int count = std::min(static_cast<int>(traj.positions.size()), max_points);
        for (int i = 0; i < count; ++i) {
            for (int j = 0; j < 6; ++j) {
                out_positions[i * 6 + j] = traj.positions[static_cast<size_t>(i)][static_cast<size_t>(j)];
                if (out_velocities && i < static_cast<int>(traj.velocities.size())) {
                    out_velocities[i * 6 + j] = traj.velocities[static_cast<size_t>(i)][static_cast<size_t>(j)];
                }
                if (out_accelerations && i < static_cast<int>(traj.accelerations.size())) {
                    out_accelerations[i * 6 + j] = traj.accelerations[static_cast<size_t>(i)][static_cast<size_t>(j)];
                }
            }
            out_time_stamps[i] = traj.time_stamps[static_cast<size_t>(i)];
        }
        *out_num_points = count;
        return 1;
    } catch (...) {
        *out_num_points = 0;
        return 0;
    }
}

int vamp_r2000ic_warmup_roadmap(VampPlannerHandle handle, double warmup_time) {
    if (!handle) return 0;
    auto* planner = static_cast<vamp_r2000ic::VampR2000icPlanner*>(handle);
    try {
        planner->warmupRoadmap(warmup_time);
        return 1;
    } catch (...) {
        return 0;
    }
}

int vamp_r2000ic_add_attached_spheres(VampPlannerHandle handle,
                                      const char* name,
                                      int link_index,
                                      const double* spheres_xyzr,
                                      int sphere_count) {
    if (!handle || !name || !spheres_xyzr || sphere_count <= 0) return 0;
    auto* planner = static_cast<vamp_r2000ic::VampR2000icPlanner*>(handle);
    try {
        std::vector<vamp_r2000ic::Sphere> list;
        list.reserve(sphere_count);
        for (int i = 0; i < sphere_count; ++i) {
            float x = static_cast<float>(spheres_xyzr[i * 4 + 0]);
            float y = static_cast<float>(spheres_xyzr[i * 4 + 1]);
            float z = static_cast<float>(spheres_xyzr[i * 4 + 2]);
            float r = static_cast<float>(spheres_xyzr[i * 4 + 3]);
            list.emplace_back(x, y, z, r);
        }
        planner->addAttachedSpheres(std::string(name), link_index, list);
        return 1;
    } catch (...) {
        return 0;
    }
}

int vamp_r2000ic_remove_attached_spheres(VampPlannerHandle handle, const char* name) {
    if (!handle || !name) return 0;
    auto* planner = static_cast<vamp_r2000ic::VampR2000icPlanner*>(handle);
    try {
        return planner->removeAttachedSpheres(std::string(name)) ? 1 : 0;
    } catch (...) {
        return 0;
    }
}

int vamp_r2000ic_clear_attached_spheres(VampPlannerHandle handle) {
    if (!handle) return 0;
    auto* planner = static_cast<vamp_r2000ic::VampR2000icPlanner*>(handle);
    try {
        planner->clearAttachedSpheres();
        return 1;
    } catch (...) {
        return 0;
    }
}
