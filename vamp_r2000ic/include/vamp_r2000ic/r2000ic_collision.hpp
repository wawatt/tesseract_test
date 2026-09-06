#pragma once

#include "simd_defs.hpp"
#include "r2000ic_simd_fk.hpp"
#include <vector>
#include <string>
#include <unordered_map>
#include <set>
#include <algorithm>
#include <cctype>
#include <iostream>
#include <tinyxml2.h>

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
 * Supports dynamic Allowed Collision Matrix (ACM) from SRDF and runtime whitelist modifications.
 */
class CollisionChecker {
public:
    CollisionChecker() {
        initDefaultSelfCollisionPairs();
    }

    static int mapLinkNameToIndex(const std::string& name) {
        std::string s = name;
        std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (s == "base_link" || s == "base" || s == "link_0" || s == "link0") return 0;
        if (s == "j1_link" || s == "j1" || s == "link_1" || s == "link1") return 1;
        if (s == "j2_link" || s == "j2" || s == "link_2" || s == "link2") return 2;
        if (s == "j3_link" || s == "j3" || s == "link_3" || s == "link3") return 3;
        if (s == "j4_link" || s == "j4" || s == "link_4" || s == "link4") return 4;
        if (s == "j5_link" || s == "j5" || s == "link_5" || s == "link5") return 5;
        if (s == "j6_link" || s == "j6" || s == "link_6" || s == "link6" || s == "flange" || s == "tool0") return 6;

        // Substring heuristics
        if (s.find("tool") != std::string::npos || s.find("flange") != std::string::npos ||
            s.find("j6") != std::string::npos || s.find("link_6") != std::string::npos) return 6;
        if (s.find("j5") != std::string::npos || s.find("link_5") != std::string::npos) return 5;
        if (s.find("j4") != std::string::npos || s.find("link_4") != std::string::npos) return 4;
        if (s.find("j3") != std::string::npos || s.find("link_3") != std::string::npos) return 3;
        if (s.find("j2") != std::string::npos || s.find("link_2") != std::string::npos) return 2;
        if (s.find("j1") != std::string::npos || s.find("link_1") != std::string::npos) return 1;
        if (s.find("base") != std::string::npos) return 0;

        return -1; // External obstacle or unknown link
    }

    /**
     * @brief Parses an SRDF file to configure Allowed Collision Matrix (ACM).
     * @param srdf_path Path to the SRDF XML configuration file.
     * @return true if successfully loaded and parsed.
     */
    bool loadSRDF(const std::string& srdf_path) {
        if (srdf_path.empty()) {
            std::cerr << "[VAMP CollisionChecker] Empty SRDF path provided." << std::endl;
            return false;
        }

        tinyxml2::XMLDocument doc;
        tinyxml2::XMLError err = doc.LoadFile(srdf_path.c_str());
        if (err != tinyxml2::XML_SUCCESS) {
            std::cerr << "[VAMP CollisionChecker] Failed to load SRDF file '" << srdf_path 
                      << "': tinyxml2 error code " << err << std::endl;
            return false;
        }

        auto* root = doc.FirstChildElement("robot");
        if (!root) {
            std::cerr << "[VAMP CollisionChecker] Root <robot> element not found in " << srdf_path << std::endl;
            return false;
        }

        std::set<std::pair<int, int>> disabled_pairs;
        int parsed_count = 0;

        for (auto* el = root->FirstChildElement("disable_collisions"); el != nullptr; el = el->NextSiblingElement("disable_collisions")) {
            const char* l1_attr = el->Attribute("link1");
            const char* l2_attr = el->Attribute("link2");
            if (!l1_attr || !l2_attr) continue;

            std::string l1(l1_attr);
            std::string l2(l2_attr);

            int idx1 = mapLinkNameToIndex(l1);
            int idx2 = mapLinkNameToIndex(l2);

            if (idx1 >= 0 && idx2 >= 0) {
                // Robot self-collision exemption
                if (idx1 > idx2) std::swap(idx1, idx2);
                disabled_pairs.insert({idx1, idx2});
                parsed_count++;
            } else if (idx1 >= 0 && idx2 < 0) {
                // l1 is robot link, l2 is external obstacle
                obstacle_exempt_masks_[l2] |= (1u << idx1);
                parsed_count++;
            } else if (idx1 < 0 && idx2 >= 0) {
                // l1 is external obstacle, l2 is robot link
                obstacle_exempt_masks_[l1] |= (1u << idx2);
                parsed_count++;
            }
        }

        std::cout << "[VAMP CollisionChecker] Successfully parsed SRDF: '" << srdf_path 
                  << "' (" << parsed_count << " disabled collision rules loaded)." << std::endl;

        rebuildSelfCollisionPairs(disabled_pairs);
        rebuildFlatBoxes();
        return true;
    }

    /**
     * @brief Dynamically modifies Allowed Collision Matrix (ACM) whitelist for two links or obstacles.
     * @param link1 Robot link name or obstacle name
     * @param link2 Robot link name or obstacle name
     * @param allowed true to exempt from collision checking, false to restore collision checking
     * @return true if successfully processed
     */
    bool setAllowedCollision(const std::string& link1, const std::string& link2, bool allowed) {
        int idx1 = mapLinkNameToIndex(link1);
        int idx2 = mapLinkNameToIndex(link2);

        if (idx1 >= 0 && idx2 >= 0) {
            // Both are robot links
            if (idx1 > idx2) std::swap(idx1, idx2);
            if (allowed) {
                disabled_self_pairs_.insert({idx1, idx2});
            } else {
                disabled_self_pairs_.erase({idx1, idx2});
            }
            rebuildSelfCollisionPairs(disabled_self_pairs_);
            return true;
        }

        // Link vs obstacle
        if (idx1 >= 0 && idx2 < 0) {
            // idx1 is robot link, link2 is obstacle
            if (allowed) {
                obstacle_exempt_masks_[link2] |= (1u << idx1);
            } else {
                obstacle_exempt_masks_[link2] &= ~(1u << idx1);
            }
            updateObstacleMask(link2);
            return true;
        } else if (idx1 < 0 && idx2 >= 0) {
            // link1 is obstacle, idx2 is robot link
            if (allowed) {
                obstacle_exempt_masks_[link1] |= (1u << idx2);
            } else {
                obstacle_exempt_masks_[link1] &= ~(1u << idx2);
            }
            updateObstacleMask(link1);
            return true;
        }

        return true;
    }

    uint32_t getObstacleExemptMask(const std::string& name) const {
        auto it = obstacle_exempt_masks_.find(name);
        return (it != obstacle_exempt_masks_.end()) ? it->second : 0u;
    }

    const std::vector<uint32_t>& getFlatBoxMasks() const {
        return flat_box_masks_;
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
        flat_box_masks_.clear();
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
     * Supports Bitmask O(1) link-level exemption for Allowed Collision Matrix.
     * @param joints 6 joint values
     * @param margin safety margin (meters)
     * @param check_self if true, checks self-collision among non-exempt links
     * @return true if collision detected, false if safe.
     */
    bool checkCollision(const double joints[6], float margin = 0.0f, bool check_self = true) const {
        std::vector<Sphere> world_spheres;
        std::vector<int> sphere_links;
        kinematics_.computeWorldSpheres(joints, world_spheres, &sphere_links);

        // 1. Check against environmental boxes (flat contiguous vector for AVX2 speed)
        size_t num_boxes = flat_boxes_.size();
        for (size_t b = 0; b < num_boxes; ++b) {
            uint32_t mask = (b < flat_box_masks_.size()) ? flat_box_masks_[b] : 0u;
            const auto& box = flat_boxes_[b];
            for (size_t s = 0; s < world_spheres.size(); ++s) {
                if (mask != 0 && ((mask >> sphere_links[s]) & 1u)) {
                    continue; // link is exempt by ACM whitelist
                }
                if (check_sphere_aabb_collision(world_spheres[s], box, margin)) {
                    return true;
                }
            }
        }

        // 2. Check against environmental spheres
        for (const auto& kv : spheres_) {
            uint32_t mask = getObstacleExemptMask(kv.first);
            const Sphere& obs = kv.second;
            for (size_t s = 0; s < world_spheres.size(); ++s) {
                if (mask != 0 && ((mask >> sphere_links[s]) & 1u)) {
                    continue; // link is exempt by ACM whitelist
                }
                if (check_sphere_sphere_collision(world_spheres[s], obs, margin)) {
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
        flat_box_masks_.clear();
        size_t total = 0;
        for (const auto& kv : obstacle_boxes_) {
            total += kv.second.size();
        }
        flat_boxes_.reserve(total);
        flat_box_masks_.reserve(total);
        for (const auto& kv : obstacle_boxes_) {
            uint32_t mask = getObstacleExemptMask(kv.first);
            for (const auto& box : kv.second) {
                flat_boxes_.push_back(box);
                flat_box_masks_.push_back(mask);
            }
        }
    }

    void updateObstacleMask(const std::string& name) {
        if (obstacle_boxes_.find(name) != obstacle_boxes_.end()) {
            rebuildFlatBoxes();
        }
    }

    void rebuildSelfCollisionPairs(const std::set<std::pair<int, int>>& disabled_pairs) {
        active_self_collision_pairs_.clear();
        disabled_self_pairs_ = disabled_pairs;

        const auto& local_spheres = kinematics_.getLocalSpheres();
        for (size_t i = 0; i < local_spheres.size(); ++i) {
            for (size_t j = i + 1; j < local_spheres.size(); ++j) {
                int link_i = local_spheres[i].link_index;
                int link_j = local_spheres[j].link_index;
                if (link_i == link_j) continue; // Same link
                if (link_i > link_j) std::swap(link_i, link_j);

                // Check if this link pair is disabled in SRDF ACM
                if (disabled_self_pairs_.find({link_i, link_j}) == disabled_self_pairs_.end()) {
                    active_self_collision_pairs_.emplace_back(static_cast<int>(i), static_cast<int>(j));
                }
            }
        }
    }

    void initDefaultSelfCollisionPairs() {
        // Default Fanuc R-2000iC/165F disabled pairs according to official SRDF ACM:
        // Adjacent: (0,1), (1,2), (2,3), (3,4), (4,5), (5,6)
        // Separated by distance: (0,2), (0,3), (1,3), (2,4), (3,5), (3,6), (4,6)
        std::set<std::pair<int, int>> default_disabled = {
            {0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 5}, {5, 6},
            {0, 2}, {0, 3}, {1, 3}, {2, 4}, {3, 5}, {3, 6}, {4, 6}
        };
        rebuildSelfCollisionPairs(default_disabled);
    }

    R2000icKinematics kinematics_;
    std::unordered_map<std::string, std::vector<AABB>> obstacle_boxes_;
    std::vector<AABB> flat_boxes_;
    std::vector<uint32_t> flat_box_masks_;
    std::unordered_map<std::string, Sphere> spheres_;
    std::unordered_map<std::string, uint32_t> obstacle_exempt_masks_;
    std::set<std::pair<int, int>> disabled_self_pairs_;
    std::vector<std::pair<int, int>> active_self_collision_pairs_;
};

} // namespace vamp_r2000ic
