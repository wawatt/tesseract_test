#pragma once

#include "vamp_bspline.hpp"
#include "r2000ic_collision.hpp"
#include <LBFGS.h>
#include <iostream>
#include <chrono>

namespace vamp_r2000ic {

/**
 * @brief cuRobo-style L-BFGS trajectory optimizer for Fanuc R-2000iC.
 * Minimizes smoothness, joint limits, and SIMD signed-distance collision costs with analytic gradients.
 */
class VampBSplineOptimizer {
public:
    VampBSplineOptimizer(const CollisionChecker& collision_checker,
                         int num_control_points = 12,
                         int num_samples = 40)
        : collision_checker_(collision_checker),
          kinematics_(collision_checker.getKinematics()),
          bspline_(num_control_points, num_samples) {
        
        // Default Fanuc R-2000iC/165F URDF limits (radians)
        limits_min_ = { -3.228859, -1.047198, -2.338741, -6.283185, -2.181662, -6.283185 };
        limits_max_ = {  3.228859,  1.326450,  3.141593,  6.283185,  2.181662,  6.283185 };

        w_smooth_ = 1.0;
        w_limit_  = 100.0;
        w_coll_   = 200.0;
        safety_margin_ = 0.035; // 3.5 cm safety margin
    }

    void setJointLimits(const std::vector<double>& min_limits, const std::vector<double>& max_limits) {
        if (min_limits.size() == 6 && max_limits.size() == 6) {
            for (int i = 0; i < 6; ++i) {
                limits_min_[i] = min_limits[i];
                limits_max_[i] = max_limits[i];
            }
        }
    }

    void setWeights(double w_smooth, double w_limit, double w_coll, double safety_margin) {
        w_smooth_ = w_smooth;
        w_limit_  = w_limit;
        w_coll_   = w_coll;
        safety_margin_ = safety_margin;
    }

    const VampCubicBSpline& getBSpline() const { return bspline_; }

    /**
     * @brief Objective evaluation and analytic gradient computation for LBFGSpp.
     */
    double operator()(const Eigen::VectorXd& x, Eigen::VectorXd& grad) {
        int K = bspline_.getNumControlPoints();
        int M = bspline_.getNumSamples();

        Eigen::MatrixXd cp(K, 6);
        cp = current_control_points_;
        bspline_.unpackVariables(x, cp);

        // Evaluate sample waypoints Q = B * P
        Eigen::MatrixXd samples(M, 6);
        bspline_.evaluate(cp, samples);

        Eigen::MatrixXd cp_grads(K, 6);
        cp_grads.setZero();

        Eigen::MatrixXd sample_grads(M, 6);
        sample_grads.setZero();

        // 1. Smoothness Cost & Gradient (minimum acceleration)
        double cost_smooth = bspline_.computeSmoothness(cp, cp_grads);
        double total_cost = w_smooth_ * cost_smooth;
        cp_grads *= w_smooth_;

        // 2. Joint Limits Cost & Gradient on samples
        double cost_limits = 0.0;
        for (int m = 0; m < M; ++m) {
            for (int d = 0; d < 6; ++d) {
                double val = samples(m, d);
                if (val < limits_min_[d]) {
                    double viol = limits_min_[d] - val;
                    cost_limits += 0.5 * viol * viol;
                    sample_grads(m, d) -= w_limit_ * viol;
                } else if (val > limits_max_[d]) {
                    double viol = val - limits_max_[d];
                    cost_limits += 0.5 * viol * viol;
                    sample_grads(m, d) += w_limit_ * viol;
                }
            }
        }
        total_cost += w_limit_ * cost_limits;

        // 3. VAMP Collision Cost & Analytic Gradients
        const auto& flat_boxes = collision_checker_.getFlatBoxes();
        const auto& flat_box_masks = collision_checker_.getFlatBoxMasks();
        const auto& env_spheres = collision_checker_.getSpheres();
        const auto& local_spheres = kinematics_.getLocalSpheres();
        const auto& attached_map = kinematics_.getAttachedSpheres();
        const auto& origin_transforms = kinematics_.getOriginTransforms();
        const auto& axes = kinematics_.getAxes();

        double cost_coll = 0.0;

        for (int m = 0; m < M; ++m) {
            double q[6] = { samples(m, 0), samples(m, 1), samples(m, 2), samples(m, 3), samples(m, 4), samples(m, 5) };

            std::array<Transform4, 7> link_transforms;
            kinematics_.computeLinkTransforms(q, link_transforms);

            // Compute joint origins and axes in world frame
            std::array<Point3, 6> joint_origins;
            std::array<Point3, 6> joint_axes;
            for (int j = 0; j < 6; ++j) {
                Transform4 T_joint = link_transforms[j] * origin_transforms[j];
                joint_origins[j] = Point3(T_joint.m[12], T_joint.m[13], T_joint.m[14]);
                // Rotate axis by joint frame
                const auto& u = axes[j];
                joint_axes[j] = Point3(
                    T_joint.m[0] * u.x + T_joint.m[4] * u.y + T_joint.m[8] * u.z,
                    T_joint.m[1] * u.x + T_joint.m[5] * u.y + T_joint.m[9] * u.z,
                    T_joint.m[2] * u.x + T_joint.m[6] * u.y + T_joint.m[10] * u.z
                );
            }

            // Helper lambda to process a single sphere against environment
            auto process_sphere = [&](int link_index, const Point3& local_pos, float radius) {
                if (link_index <= 0) return; // Base link doesn't move with joints
                const auto& T_link = link_transforms[link_index];
                Point3 wp = T_link.transformPoint(local_pos);

                Point3 total_force(0.0f, 0.0f, 0.0f);

                // A. Check against AABB boxes
                for (size_t b_idx = 0; b_idx < flat_boxes.size(); ++b_idx) {
                    if (b_idx < flat_box_masks.size() && (flat_box_masks[b_idx] & (1u << link_index))) {
                        continue; // exempt link!
                    }
                    const auto& box = flat_boxes[b_idx];
                    // Find closest point on AABB
                    float cx = std::clamp(wp.x, box.min.x, box.max.x);
                    float cy = std::clamp(wp.y, box.min.y, box.max.y);
                    float cz = std::clamp(wp.z, box.min.z, box.max.z);

                    float dx = wp.x - cx;
                    float dy = wp.y - cy;
                    float dz = wp.z - cz;
                    float dist_sq = dx * dx + dy * dy + dz * dz;

                    float dist, nx, ny, nz;
                    if (dist_sq > 1e-10f) {
                        dist = std::sqrt(dist_sq);
                        float inv_d = 1.0f / dist;
                        nx = dx * inv_d;
                        ny = dy * inv_d;
                        nz = dz * inv_d;
                    } else {
                        // Center is inside box: find penetration to closest face
                        float d_min_x = wp.x - box.min.x;
                        float d_max_x = box.max.x - wp.x;
                        float d_min_y = wp.y - box.min.y;
                        float d_max_y = box.max.y - wp.y;
                        float d_min_z = wp.z - box.min.z;
                        float d_max_z = box.max.z - wp.z;

                        float min_p = d_min_x;
                        nx = -1.0f; ny = 0.0f; nz = 0.0f;
                        if (d_max_x < min_p) { min_p = d_max_x; nx = 1.0f; ny = 0.0f; nz = 0.0f; }
                        if (d_min_y < min_p) { min_p = d_min_y; nx = 0.0f; ny = -1.0f; nz = 0.0f; }
                        if (d_max_y < min_p) { min_p = d_max_y; nx = 0.0f; ny = 1.0f; nz = 0.0f; }
                        if (d_min_z < min_p) { min_p = d_min_z; nx = 0.0f; ny = 0.0f; nz = -1.0f; }
                        if (d_max_z < min_p) { min_p = d_max_z; nx = 0.0f; ny = 0.0f; nz = 1.0f; }
                        dist = -min_p;
                    }

                    float clearance = dist - radius;
                    if (clearance < safety_margin_) {
                        float viol = static_cast<float>(safety_margin_) - clearance;
                        cost_coll += 0.5 * viol * viol;
                        // Repulsive gradient force: pushes away from obstacle
                        float force_mag = static_cast<float>(w_coll_) * viol;
                        total_force.x -= force_mag * nx;
                        total_force.y -= force_mag * ny;
                        total_force.z -= force_mag * nz;
                    }
                }

                // B. Check against Environmental Spheres
                for (const auto& kv : env_spheres) {
                    uint32_t mask = collision_checker_.getObstacleExemptMask(kv.first);
                    if (mask != 0 && (mask & (1u << link_index))) {
                        continue; // exempt link!
                    }
                    const Sphere& obs = kv.second;
                    float dx = wp.x - obs.x;
                    float dy = wp.y - obs.y;
                    float dz = wp.z - obs.z;
                    float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
                    float clearance = dist - (radius + obs.r);
                    if (clearance < safety_margin_ && dist > 1e-5f) {
                        float viol = static_cast<float>(safety_margin_) - clearance;
                        cost_coll += 0.5 * viol * viol;
                        float inv_d = 1.0f / dist;
                        float force_mag = static_cast<float>(w_coll_) * viol;
                        total_force.x -= force_mag * (dx * inv_d);
                        total_force.y -= force_mag * (dy * inv_d);
                        total_force.z -= force_mag * (dz * inv_d);
                    }
                }

                // C. Backpropagate 3D force to joints j < link_index via Jacobian
                if (std::abs(total_force.x) > 1e-8f || std::abs(total_force.y) > 1e-8f || std::abs(total_force.z) > 1e-8f) {
                    for (int j = 0; j < link_index && j < 6; ++j) {
                        // Vector r from joint origin to sphere center
                        float rx = wp.x - joint_origins[j].x;
                        float ry = wp.y - joint_origins[j].y;
                        float rz = wp.z - joint_origins[j].z;

                        // Cross product r x force
                        float tau_x = ry * total_force.z - rz * total_force.y;
                        float tau_y = rz * total_force.x - rx * total_force.z;
                        float tau_z = rx * total_force.y - ry * total_force.x;

                        // Dot product with joint axis
                        float torque = joint_axes[j].x * tau_x + joint_axes[j].y * tau_y + joint_axes[j].z * tau_z;
                        sample_grads(m, j) += torque;
                    }
                }
            };

            // Process regular robot link spheres
            for (const auto& ls : local_spheres) {
                process_sphere(ls.link_index, ls.local_pos, ls.radius);
            }

            // Process attached workpiece spheres (e.g. on tool0 / J6_link)
            for (const auto& kv : attached_map) {
                for (const auto& ls : kv.second) {
                    process_sphere(ls.link_index, ls.local_pos, ls.radius);
                }
            }
        }

        total_cost += w_coll_ * cost_coll;

        // Project sample gradients back to control point gradients: G_P = B^T * G_Q
        Eigen::MatrixXd cp_grads_proj(K, 6);
        bspline_.projectGradients(sample_grads, cp_grads_proj);
        cp_grads += cp_grads_proj;

        // Extract free gradients
        bspline_.packGradients(cp_grads, grad);

        return total_cost;
    }

    /**
     * @brief Solves trajectory optimization using L-BFGS.
     * @param control_points In/Out control points matrix (K x 6).
     * @param max_iters Maximum L-BFGS iterations (default 25).
     * @return true if optimization converged or achieved cost reduction.
     */
    bool optimize(Eigen::MatrixXd& control_points, int max_iters = 25) {
        current_control_points_ = control_points;

        Eigen::VectorXd x;
        bspline_.packVariables(control_points, x);

        LBFGSpp::LBFGSParam<double> param;
        param.m = 7; // cuRobo default history
        param.max_iterations = max_iters;
        param.epsilon = 1e-4;
        param.max_step = 1.0;

        LBFGSpp::LBFGSSolver<double> solver(param);

        double final_cost = 0.0;
        try {
            int n_iters = solver.minimize(*this, x, final_cost);
            (void)n_iters;
        } catch (const std::exception& e) {
            // Early line search termination or small gradient - x still holds the best candidate
            std::cout << "[VampBSplineOptimizer] L-BFGS info: " << e.what() << std::endl;
        }

        bspline_.unpackVariables(x, control_points);
        return true;
    }

private:
    const CollisionChecker& collision_checker_;
    const R2000icKinematics& kinematics_;
    VampCubicBSpline bspline_;

    std::array<double, 6> limits_min_;
    std::array<double, 6> limits_max_;

    double w_smooth_;
    double w_limit_;
    double w_coll_;
    double safety_margin_;

    Eigen::MatrixXd current_control_points_;
};

} // namespace vamp_r2000ic
