#pragma once

#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include "robot_planner.h"

#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif

namespace robot_planner {

/**
 * @brief Dynamic loader for vamp_r2000ic.dll.
 * Loads the SIMD-accelerated planning library at runtime via LoadLibraryA/GetProcAddress.
 */
class VampDynamicLoader {
public:
    typedef void* (*FnCreate)(void);
    typedef void (*FnDestroy)(void*);
    typedef int (*FnInit)(void*);
    typedef int (*FnAddBox)(void*, const char*, double, double, double, double, double, double);
    typedef int (*FnAddBoxes)(void*, const char*, const double*, int);
    typedef int (*FnRemoveObstacle)(void*, const char*);
    typedef int (*FnCheckCollision)(void*, const double*);
    typedef int (*FnPlanFreespace)(void*, const double*, const double*, double*, int*, int, double, double, double, const char*);
    typedef int (*FnPlanTrajectory)(void*, const double*, const double*, double*, double*, double*, double*, int*, int, double, double, double, double, double, const char*);
    typedef int (*FnParameterize)(void*, const double*, int, double*, double*, double*, double*, int*, int, double, double, double);
    typedef int (*FnSetJointOrigins)(void*, const double*, const double*, const double*);
    typedef int (*FnSetLimits)(void*, const double*, const double*);
    typedef int (*FnWarmupRoadmap)(void*, double);
    typedef int (*FnAddAttachedSpheres)(void*, const char*, int, const double*, int);
    typedef int (*FnRemoveAttachedSpheres)(void*, const char*);
    typedef int (*FnClearAttachedSpheres)(void*);
    typedef int (*FnLoadSRDF)(void*, const char*);
    typedef int (*FnSetAllowedCollision)(void*, const char*, const char*, int);

    VampDynamicLoader() = default;

    ~VampDynamicLoader() {
        unload();
    }

    bool isLoaded() const {
        return handle_ != nullptr;
    }

    /**
     * @brief Attempts to dynamically locate and load robot-specific VAMP accelerator DLL.
     * @param explicit_path User provided DLL path
     * @param robot_name Robot identifier (e.g., "r2000ic_165f")
     */
    bool load(const std::string& explicit_path = "", const std::string& robot_name = "") {
        if (isLoaded()) return true;

        std::vector<std::string> candidate_paths;
        if (!explicit_path.empty()) {
            candidate_paths.push_back(explicit_path);
        }

        std::vector<std::string> dll_names;
        if (!robot_name.empty()) {
            dll_names.push_back("vamp_" + robot_name + ".dll");
            // If robot_name contains sub-ident (e.g. r2000ic_165f -> r2000ic)
            size_t under = robot_name.find('_');
            if (under != std::string::npos) {
                dll_names.push_back("vamp_" + robot_name.substr(0, under) + ".dll");
            }
        }
        dll_names.push_back("vamp_r2000ic.dll");

#ifdef _WIN32
        // Get directory of current running executable
        char exe_path[MAX_PATH] = {0};
        std::string exe_dir = "";
        if (GetModuleFileNameA(NULL, exe_path, MAX_PATH)) {
            std::string dir(exe_path);
            size_t pos = dir.find_last_of("\\/");
            if (pos != std::string::npos) {
                exe_dir = dir.substr(0, pos + 1);
            }
        }

        for (const auto& dll : dll_names) {
            if (!exe_dir.empty()) {
                candidate_paths.push_back(exe_dir + dll);
            }
            candidate_paths.push_back(dll);
            candidate_paths.push_back("workspace/Release/" + dll);
            candidate_paths.push_back("../workspace/Release/" + dll);
            candidate_paths.push_back("../../workspace/Release/" + dll);
        }

        for (const auto& path : candidate_paths) {
            hModule_ = LoadLibraryA(path.c_str());
            if (hModule_) {
                resolved_path_ = path;
                break;
            }
        }

        if (!hModule_) {
            return false;
        }

        fnCreate_ = (FnCreate)GetProcAddress(hModule_, "vamp_r2000ic_create");
        fnDestroy_ = (FnDestroy)GetProcAddress(hModule_, "vamp_r2000ic_destroy");
        fnInit_ = (FnInit)GetProcAddress(hModule_, "vamp_r2000ic_init");
        fnAddBox_ = (FnAddBox)GetProcAddress(hModule_, "vamp_r2000ic_add_box");
        fnAddBoxes_ = (FnAddBoxes)GetProcAddress(hModule_, "vamp_r2000ic_add_boxes");
        fnRemoveObstacle_ = (FnRemoveObstacle)GetProcAddress(hModule_, "vamp_r2000ic_remove_obstacle");
        fnCheckCollision_ = (FnCheckCollision)GetProcAddress(hModule_, "vamp_r2000ic_check_collision");
        fnPlanFreespace_ = (FnPlanFreespace)GetProcAddress(hModule_, "vamp_r2000ic_plan_freespace");
        fnPlanTrajectory_ = (FnPlanTrajectory)GetProcAddress(hModule_, "vamp_r2000ic_plan_trajectory");
        fnParameterize_ = (FnParameterize)GetProcAddress(hModule_, "vamp_r2000ic_parameterize");
        fnSetJointOrigins_ = (FnSetJointOrigins)GetProcAddress(hModule_, "vamp_r2000ic_set_joint_origins");
        fnSetLimits_ = (FnSetLimits)GetProcAddress(hModule_, "vamp_r2000ic_set_limits");
        fnWarmupRoadmap_ = (FnWarmupRoadmap)GetProcAddress(hModule_, "vamp_r2000ic_warmup_roadmap");
        fnAddAttachedSpheres_ = (FnAddAttachedSpheres)GetProcAddress(hModule_, "vamp_r2000ic_add_attached_spheres");
        fnRemoveAttachedSpheres_ = (FnRemoveAttachedSpheres)GetProcAddress(hModule_, "vamp_r2000ic_remove_attached_spheres");
        fnClearAttachedSpheres_ = (FnClearAttachedSpheres)GetProcAddress(hModule_, "vamp_r2000ic_clear_attached_spheres");
        fnLoadSRDF_ = (FnLoadSRDF)GetProcAddress(hModule_, "vamp_r2000ic_load_srdf");
        fnSetAllowedCollision_ = (FnSetAllowedCollision)GetProcAddress(hModule_, "vamp_r2000ic_set_allowed_collision");

        if (!fnCreate_ || !fnDestroy_ || !fnInit_ || !fnAddBox_ || !fnRemoveObstacle_ || !fnCheckCollision_ || !fnPlanFreespace_) {
            unload();
            return false;
        }
#endif

        if (fnCreate_) {
            handle_ = fnCreate_();
            if (handle_ && fnInit_) {
                if (fnInit_(handle_) != 1) {
                    unload();
                    return false;
                }
            }
        }

        return isLoaded();
    }

    void unload() {
        if (handle_ && fnDestroy_) {
            fnDestroy_(handle_);
            handle_ = nullptr;
        }
#ifdef _WIN32
        if (hModule_) {
            FreeLibrary(hModule_);
            hModule_ = nullptr;
        }
#endif
        fnCreate_ = nullptr;
        fnDestroy_ = nullptr;
        fnInit_ = nullptr;
        fnAddBox_ = nullptr;
        fnAddBoxes_ = nullptr;
        fnRemoveObstacle_ = nullptr;
        fnCheckCollision_ = nullptr;
        fnPlanFreespace_ = nullptr;
        fnPlanTrajectory_ = nullptr;
        fnParameterize_ = nullptr;
        fnSetJointOrigins_ = nullptr;
        fnWarmupRoadmap_ = nullptr;
        fnAddAttachedSpheres_ = nullptr;
        fnRemoveAttachedSpheres_ = nullptr;
        fnClearAttachedSpheres_ = nullptr;
    }

    const std::string& getResolvedPath() const {
        return resolved_path_;
    }

    bool setJointOrigins(const std::vector<double>& origins_xyz,
                         const std::vector<double>& origins_rpy,
                         const std::vector<double>& axes_xyz) {
        if (!isLoaded() || !fnSetJointOrigins_) return false;
        if (origins_xyz.size() < 18 || origins_rpy.size() < 18 || axes_xyz.size() < 18) return false;
        return fnSetJointOrigins_(handle_, origins_xyz.data(), origins_rpy.data(), axes_xyz.data()) == 1;
    }

    bool addBox(const std::string& name, double x, double y, double z, double dx, double dy, double dz) {
        if (!isLoaded() || !fnAddBox_) return false;
        return fnAddBox_(handle_, name.c_str(), x, y, z, dx, dy, dz) == 1;
    }

    bool addBoxes(const std::string& name, const std::vector<double>& aabb_min_max_array) {
        if (!isLoaded() || !fnAddBoxes_ || aabb_min_max_array.empty()) return false;
        int box_count = static_cast<int>(aabb_min_max_array.size() / 6);
        if (box_count <= 0) return false;
        return fnAddBoxes_(handle_, name.c_str(), aabb_min_max_array.data(), box_count) == 1;
    }

    bool removeObstacle(const std::string& name) {
        if (!isLoaded() || !fnRemoveObstacle_) return false;
        return fnRemoveObstacle_(handle_, name.c_str()) == 1;
    }

    bool checkCollision(const std::vector<double>& joints) {
        if (!isLoaded() || !fnCheckCollision_ || joints.size() < 6) return true;
        return fnCheckCollision_(handle_, joints.data()) == 1;
    }

    bool warmupRoadmap(double warmup_time = 0.3) {
        if (!isLoaded() || !fnWarmupRoadmap_) return false;
        return fnWarmupRoadmap_(handle_, warmup_time) == 1;
    }

    bool setLimits(const std::vector<double>& vel_limits, const std::vector<double>& acc_limits) {
        if (!isLoaded() || !fnSetLimits_ || vel_limits.size() < 6 || acc_limits.size() < 6) return false;
        return fnSetLimits_(handle_, vel_limits.data(), acc_limits.data()) == 1;
    }

    bool loadSRDF(const std::string& srdf_path) {
        if (!isLoaded() || !fnLoadSRDF_) return false;
        return fnLoadSRDF_(handle_, srdf_path.c_str()) == 1;
    }

    bool setAllowedCollision(const std::string& link1, const std::string& link2, bool allowed) {
        if (!isLoaded() || !fnSetAllowedCollision_) return false;
        return fnSetAllowedCollision_(handle_, link1.c_str(), link2.c_str(), allowed ? 1 : 0) == 1;
    }

    bool addAttachedSpheres(const std::string& name, int link_index, const double* spheres_xyzr, int sphere_count) {
        if (!isLoaded() || !fnAddAttachedSpheres_ || !spheres_xyzr || sphere_count <= 0) return false;
        return fnAddAttachedSpheres_(handle_, name.c_str(), link_index, spheres_xyzr, sphere_count) == 1;
    }

    bool removeAttachedSpheres(const std::string& name) {
        if (!isLoaded() || !fnRemoveAttachedSpheres_) return false;
        return fnRemoveAttachedSpheres_(handle_, name.c_str()) == 1;
    }

    bool clearAttachedSpheres() {
        if (!isLoaded() || !fnClearAttachedSpheres_) return false;
        return fnClearAttachedSpheres_(handle_) == 1;
    }

    bool planFreespace(const std::vector<double>& start,
                       const std::vector<double>& goal,
                       std::vector<std::vector<double>>& trajectory_out,
                       double timeout = 5.0,
                       double step_size = 0.02,
                       double margin = 0.025,
                       const std::string& planner_type = "PRM") {
        if (!isLoaded() || !fnPlanFreespace_ || start.size() < 6 || goal.size() < 6) {
            return false;
        }

        const int MAX_POINTS = 5000;
        std::vector<double> buffer(MAX_POINTS * 6);
        int num_points = 0;

        int ret = fnPlanFreespace_(handle_, start.data(), goal.data(), buffer.data(), &num_points, MAX_POINTS,
                                  timeout, step_size, margin, planner_type.c_str());
        if (ret != 1 || num_points <= 0) {
            return false;
        }

        trajectory_out.clear();
        trajectory_out.resize(num_points, std::vector<double>(6));
        for (int i = 0; i < num_points; ++i) {
            for (int j = 0; j < 6; ++j) {
                trajectory_out[i][j] = buffer[i * 6 + j];
            }
        }
        return true;
    }

    bool planTrajectory(const std::vector<double>& start,
                        const std::vector<double>& goal,
                        JointTrajectory& trajectory_out,
                        double max_vel_scaling = 1.0,
                        double max_acc_scaling = 1.0,
                        double timeout = 5.0,
                        double step_size = 0.02,
                        double margin = 0.025,
                        const std::string& planner_type = "PRM") {
        trajectory_out.clear();
        if (!isLoaded() || !fnPlanTrajectory_ || start.size() < 6 || goal.size() < 6) {
            return false;
        }

        const int MAX_POINTS = 5000;
        std::vector<double> buf_pos(MAX_POINTS * 6);
        std::vector<double> buf_vel(MAX_POINTS * 6);
        std::vector<double> buf_acc(MAX_POINTS * 6);
        std::vector<double> buf_time(MAX_POINTS);
        int num_points = 0;

        int ret = fnPlanTrajectory_(handle_, start.data(), goal.data(),
                                   buf_pos.data(), buf_vel.data(), buf_acc.data(), buf_time.data(),
                                   &num_points, MAX_POINTS,
                                   max_vel_scaling, max_acc_scaling,
                                   timeout, step_size, margin, planner_type.c_str());
        if (ret != 1 || num_points <= 0) {
            return false;
        }

        trajectory_out.positions.resize(num_points, std::vector<double>(6));
        trajectory_out.velocities.resize(num_points, std::vector<double>(6));
        trajectory_out.accelerations.resize(num_points, std::vector<double>(6));
        trajectory_out.time_stamps.resize(num_points);

        for (int i = 0; i < num_points; ++i) {
            for (int j = 0; j < 6; ++j) {
                trajectory_out.positions[i][j] = buf_pos[i * 6 + j];
                trajectory_out.velocities[i][j] = buf_vel[i * 6 + j];
                trajectory_out.accelerations[i][j] = buf_acc[i * 6 + j];
            }
            trajectory_out.time_stamps[i] = buf_time[i];
        }
        return true;
    }

    bool parameterize(const std::vector<std::vector<double>>& waypoints,
                      JointTrajectory& trajectory_out,
                      double max_vel_scaling = 1.0,
                      double max_acc_scaling = 1.0,
                      double sample_dt = 0.01) {
        trajectory_out.clear();
        if (!isLoaded() || !fnParameterize_ || waypoints.size() < 2) {
            return false;
        }

        const int num_waypoints = static_cast<int>(waypoints.size());
        std::vector<double> in_flat(static_cast<size_t>(num_waypoints) * 6);
        for (int i = 0; i < num_waypoints; ++i) {
            if (waypoints[static_cast<size_t>(i)].size() < 6) return false;
            for (int j = 0; j < 6; ++j) {
                in_flat[static_cast<size_t>(i) * 6 + static_cast<size_t>(j)] = waypoints[static_cast<size_t>(i)][static_cast<size_t>(j)];
            }
        }

        const int MAX_POINTS = 5000;
        std::vector<double> buf_pos(MAX_POINTS * 6);
        std::vector<double> buf_vel(MAX_POINTS * 6);
        std::vector<double> buf_acc(MAX_POINTS * 6);
        std::vector<double> buf_time(MAX_POINTS);
        int num_points = 0;

        int ret = fnParameterize_(handle_, in_flat.data(), num_waypoints,
                                  buf_pos.data(), buf_vel.data(), buf_acc.data(), buf_time.data(),
                                  &num_points, MAX_POINTS,
                                  max_vel_scaling, max_acc_scaling, sample_dt);
        if (ret != 1 || num_points <= 0) {
            return false;
        }

        trajectory_out.positions.resize(num_points, std::vector<double>(6));
        trajectory_out.velocities.resize(num_points, std::vector<double>(6));
        trajectory_out.accelerations.resize(num_points, std::vector<double>(6));
        trajectory_out.time_stamps.resize(num_points);
        for (int i = 0; i < num_points; ++i) {
            for (int j = 0; j < 6; ++j) {
                trajectory_out.positions[i][j] = buf_pos[i * 6 + j];
                trajectory_out.velocities[i][j] = buf_vel[i * 6 + j];
                trajectory_out.accelerations[i][j] = buf_acc[i * 6 + j];
            }
            trajectory_out.time_stamps[i] = buf_time[i];
        }
        return true;
    }

private:
#ifdef _WIN32
    HMODULE hModule_ = nullptr;
#endif
    void* handle_ = nullptr;
    std::string resolved_path_;

    FnCreate fnCreate_ = nullptr;
    FnDestroy fnDestroy_ = nullptr;
    FnInit fnInit_ = nullptr;
    FnAddBox fnAddBox_ = nullptr;
    FnAddBoxes fnAddBoxes_ = nullptr;
    FnRemoveObstacle fnRemoveObstacle_ = nullptr;
    FnCheckCollision fnCheckCollision_ = nullptr;
    FnPlanFreespace fnPlanFreespace_ = nullptr;
    FnPlanTrajectory fnPlanTrajectory_ = nullptr;
    FnParameterize fnParameterize_ = nullptr;
    FnSetJointOrigins fnSetJointOrigins_ = nullptr;
    FnSetLimits fnSetLimits_ = nullptr;
    FnWarmupRoadmap fnWarmupRoadmap_ = nullptr;
    FnAddAttachedSpheres fnAddAttachedSpheres_ = nullptr;
    FnRemoveAttachedSpheres fnRemoveAttachedSpheres_ = nullptr;
    FnClearAttachedSpheres fnClearAttachedSpheres_ = nullptr;
    FnLoadSRDF fnLoadSRDF_ = nullptr;
    FnSetAllowedCollision fnSetAllowedCollision_ = nullptr;
};

} // namespace robot_planner
