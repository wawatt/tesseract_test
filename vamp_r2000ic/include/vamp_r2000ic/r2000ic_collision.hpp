#pragma once

#include "simd_defs.hpp"
#include "r2000ic_simd_fk.hpp"
#include <vector>
#include <string>
#include <unordered_map>

namespace vamp_r2000ic {

struct ObstacleBox {
    std::string name;
    AABB aabb;
};

struct ObstacleSphere {
    std::string name;
    Sphere sphere;
};

/**
 * @brief Manages the collision environment (boxes, spheres) and executes SIMD collision tests.
 */
class CollisionChecker {
public:
    CollisionChecker() {
        initSelfCollisionPairs();
    }

    void addBox(const std::string& name, float min_x, float min_y, float min_z,
                float max_x, float max_y, float max_z) {
        obstacle_boxes_[name] = { AABB(Point3(min_x, min_y, min_z), Point3(max_x, max_y, max_z)) };
        rebuildFlatBoxes();
    }

    void addBoxCenterDim(const std::string& name, float cx, float cy, float cz,
                         float dx, float dy, float dz) {
        float hx = dx * 0.5f;
        float hy = dy * 0.5f;
        float hz = dz * 0.5f;
        addBox(name, cx - hx, cy - hy, cz - hz, cx + hx, cy + hy, cz + hz);
    }

    void addBoxes(const std::string& name, const std::vector<AABB>& boxes) {
        obstacle_boxes_[name] = boxes;
        rebuildFlatBoxes();
    }

    void addSphere(const std::string& name, float x, float y, float z, float r) {
        spheres_[name] = Sphere(x, y, z, r);
    }

    bool removeObstacle(const std::string& name) {
        bool removed = false;
        if (obstacle_boxes_.erase(name) > 0) {
            removed = true;
            rebuildFlatBoxes();
        }
        if (spheres_.erase(name) > 0) removed = true;
        return removed;
    }

    void clearObstacles() {
        obstacle_boxes_.clear();
        flat_boxes_.clear();
        spheres_.clear();
    }

    void setJointOrigins(const double origins_xyz[18], const double origins_rpy[18], const double axes_xyz[18]) {
        kinematics_.setJointOrigins(origins_xyz, origins_rpy, axes_xyz);
    }

    void addAttachedSpheres(const std::string& name, int link_index, const std::vector<Sphere>& spheres) {
        kinematics_.addAttachedSpheres(name, link_index, spheres);
    }

    bool removeAttachedSpheres(const std::string& name) {
        return kinematics_.removeAttachedSpheres(name);
    }

    void clearAttachedSpheres() {
        kinematics_.clearAttachedSpheres();
    }

    /**
     * @brief Checks whether the robot at given joint angles is in collision with environment or self.
     * @param joints 6 joint values
     * @param margin safety margin (meters)
     * @param check_self if true, checks self-collision among non-exempt links
     * @return true if collision detected, false if safe.
     */
    bool checkCollision(const double joints[6], float margin = 0.0f, bool check_self = true) const {
        std::vector<Sphere> world_spheres;
        kinematics_.computeWorldSpheres(joints, world_spheres);

        // 1. Check against environmental boxes (flat contiguous vector for AVX2 speed)
        for (const auto& box : flat_boxes_) {
            for (const auto& s : world_spheres) {
                if (check_sphere_aabb_collision(s, box, margin)) {
                    return true;
                }
            }
        }

        // 2. Check against environmental spheres
        for (const auto& kv : spheres_) {
            const Sphere& obs = kv.second;
            for (const auto& s : world_spheres) {
                if (check_sphere_sphere_collision(s, obs, margin)) {
                    return true;
                }
            }
        }

        // 3. Self-collision (check active pairs from Allowed Collision Matrix)
        if (check_self) {
            for (const auto& pair : active_self_collision_pairs_) {
                if (check_sphere_sphere_collision(world_spheres[pair.first], world_spheres[pair.second], margin)) {
                    return true;
                }
            }
        }

        return false;
    }

    /**
     * @brief Batch collision check for multiple joint states.
     * High throughput evaluation for sampling-based planners.
     */
    void checkCollisionBatch(const std::vector<std::array<double, 6>>& states,
                             std::vector<bool>& collision_results,
                             float margin = 0.0f) const {
        collision_results.resize(states.size());
        for (size_t i = 0; i < states.size(); ++i) {
            collision_results[i] = checkCollision(states[i].data(), margin, true);
        }
    }

    const R2000icKinematics& getKinematics() const {
        return kinematics_;
    }

    const std::vector<AABB>& getFlatBoxes() const {
        return flat_boxes_;
    }

    const std::unordered_map<std::string, Sphere>& getSpheres() const {
        return spheres_;
    }

    const std::vector<std::pair<int, int>>& getActiveSelfCollisionPairs() const {
        return active_self_collision_pairs_;
    }

    size_t getNumBoxes() const {
        return flat_boxes_.size();
    }

private:
    void rebuildFlatBoxes() {
        flat_boxes_.clear();
        size_t total = 0;
        for (const auto& kv : obstacle_boxes_) {
            total += kv.second.size();
        }
        flat_boxes_.reserve(total);
        for (const auto& kv : obstacle_boxes_) {
            flat_boxes_.insert(flat_boxes_.end(), kv.second.begin(), kv.second.end());
        }
    }

    R2000icKinematics kinematics_;
    std::unordered_map<std::string, std::vector<AABB>> obstacle_boxes_;
    std::vector<AABB> flat_boxes_;
    std::unordered_map<std::string, Sphere> spheres_;
    std::vector<std::pair<int, int>> active_self_collision_pairs_;

    void initSelfCollisionPairs() {
        // Pairs of spheres belonging to links that can collide according to SRDF ACM:
        // Only (base, J4/J5/J6), (J1, J4/J5/J6), and (J2, J5/J6) can physically collide.
        const auto& local_spheres = kinematics_.getLocalSpheres();
        for (size_t i = 0; i < local_spheres.size(); ++i) {
            for (size_t j = i + 1; j < local_spheres.size(); ++j) {
                int link_i = local_spheres[i].link_index;
                int link_j = local_spheres[j].link_index;
                if (link_i > link_j) std::swap(link_i, link_j);

                bool can_collide = false;
                // Base (0) vs Upper Arm/Wrist (4, 5, 6)
                if (link_i == 0 && (link_j == 4 || link_j == 5 || link_j == 6)) can_collide = true;
                // J1 (1) vs Upper Arm/Wrist (4, 5, 6)
                if (link_i == 1 && (link_j == 4 || link_j == 5 || link_j == 6)) can_collide = true;
                // J2 (2) vs Wrist/Flange (5, 6)
                if (link_i == 2 && (link_j == 5 || link_j == 6)) can_collide = true;

                if (can_collide) {
                    active_self_collision_pairs_.emplace_back(static_cast<int>(i), static_cast<int>(j));
                }
            }
        }
    }
};

} // namespace vamp_r2000ic
