#pragma once

#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <Eigen/Core>

namespace vamp_r2000ic {

/**
 * @brief Uniform Cubic B-Spline trajectory parameterization for 6-DOF robot.
 * Clamped triple-knot endpoints ensure exact start/goal match with zero boundary velocity and acceleration.
 */
class VampCubicBSpline {
public:
    static const int DOF = 6;

    VampCubicBSpline(int num_control_points = 12, int num_samples = 40)
        : K_(num_control_points), M_(num_samples) {
        if (K_ < 8) K_ = 8;
        if (M_ < 10) M_ = 10;
        precomputeBasis();
    }

    int getNumControlPoints() const { return K_; }
    int getNumSamples() const { return M_; }
    int getNumFreePoints() const { return K_ - 6; } // P_3 to P_{K-4}
    int getNumVariables() const { return getNumFreePoints() * DOF; }

    /**
     * @brief Initializes control points from an initial path (e.g. from PRM).
     */
    void initFromWaypoints(const std::vector<std::vector<double>>& waypoints, 
                           const std::vector<double>& start_joints, 
                           const std::vector<double>& target_joints,
                           Eigen::MatrixXd& control_points) const {
        control_points.resize(K_, DOF);

        // Clamped boundary condition: triple knot at start and target
        for (int i = 0; i < 3; ++i) {
            for (int d = 0; d < DOF; ++d) {
                control_points(i, d) = start_joints[d];
                control_points(K_ - 1 - i, d) = target_joints[d];
            }
        }

        if (waypoints.size() <= 2) {
            // Linear interpolation for intermediate free control points
            for (int i = 3; i < K_ - 3; ++i) {
                double alpha = static_cast<double>(i - 2) / (K_ - 5);
                for (int d = 0; d < DOF; ++d) {
                    control_points(i, d) = (1.0 - alpha) * start_joints[d] + alpha * target_joints[d];
                }
            }
            return;
        }

        // Compute cumulative arc-length along PRM waypoints
        std::vector<double> cum_len(waypoints.size(), 0.0);
        for (size_t i = 1; i < waypoints.size(); ++i) {
            double d = 0.0;
            for (int j = 0; j < DOF; ++j) {
                double diff = waypoints[i][j] - waypoints[i - 1][j];
                d += diff * diff;
            }
            cum_len[i] = cum_len[i - 1] + std::sqrt(d);
        }
        double total_len = cum_len.back();

        int num_free = K_ - 6;
        for (int i = 0; i < num_free; ++i) {
            double target_s = (static_cast<double>(i + 1) / (num_free + 1)) * total_len;
            // Find segment in waypoints
            size_t seg = 0;
            while (seg + 1 < cum_len.size() && cum_len[seg + 1] < target_s) {
                seg++;
            }
            double seg_len = (seg + 1 < cum_len.size()) ? (cum_len[seg + 1] - cum_len[seg]) : 0.0;
            double frac = (seg_len > 1e-6) ? (target_s - cum_len[seg]) / seg_len : 0.0;
            frac = std::clamp(frac, 0.0, 1.0);

            int cp_idx = 3 + i;
            for (int d = 0; d < DOF; ++d) {
                control_points(cp_idx, d) = (1.0 - frac) * waypoints[seg][d] + frac * waypoints[seg + 1][d];
            }
        }
    }

    /**
     * @brief Evaluates trajectory samples Q = B * P.
     */
    void evaluate(const Eigen::MatrixXd& control_points, Eigen::MatrixXd& samples) const {
        samples = B_ * control_points;
    }

    /**
     * @brief Backprojects sample gradients to control point gradients: G_P = B^T * G_Q.
     */
    void projectGradients(const Eigen::MatrixXd& sample_grads, Eigen::MatrixXd& cp_grads) const {
        cp_grads = B_.transpose() * sample_grads;
    }

    /**
     * @brief Computes analytic smoothness cost and gradient: sum_{k=1}^{K-2} ||P_{k+1} - 2P_k + P_{k-1}||^2
     */
    double computeSmoothness(const Eigen::MatrixXd& control_points, Eigen::MatrixXd& cp_grads) const {
        double cost = 0.0;
        // Acceleration finite-difference
        for (int k = 1; k < K_ - 1; ++k) {
            Eigen::VectorXd diff = control_points.row(k + 1) - 2.0 * control_points.row(k) + control_points.row(k - 1);
            cost += diff.squaredNorm();
        }

        // 5-point stencil gradient
        for (int k = 1; k < K_ - 1; ++k) {
            Eigen::VectorXd diff = control_points.row(k + 1) - 2.0 * control_points.row(k) + control_points.row(k - 1);
            // Contribution from term k
            cp_grads.row(k + 1) += 2.0 * diff;
            cp_grads.row(k)     -= 4.0 * diff;
            cp_grads.row(k - 1) += 2.0 * diff;
        }

        return cost;
    }

    /**
     * @brief Extracts free variables x (P_3 to P_{K-4}) into a flat Eigen vector.
     */
    void packVariables(const Eigen::MatrixXd& control_points, Eigen::VectorXd& x) const {
        int num_free = getNumFreePoints();
        x.resize(num_free * DOF);
        for (int i = 0; i < num_free; ++i) {
            int cp_idx = 3 + i;
            for (int d = 0; d < DOF; ++d) {
                x[i * DOF + d] = control_points(cp_idx, d);
            }
        }
    }

    /**
     * @brief Unpacks flat variable vector x back into control points matrix.
     */
    void unpackVariables(const Eigen::VectorXd& x, Eigen::MatrixXd& control_points) const {
        int num_free = getNumFreePoints();
        for (int i = 0; i < num_free; ++i) {
            int cp_idx = 3 + i;
            for (int d = 0; d < DOF; ++d) {
                control_points(cp_idx, d) = x[i * DOF + d];
            }
        }
    }

    /**
     * @brief Extracts free variable gradients from full control point gradient matrix.
     */
    void packGradients(const Eigen::MatrixXd& cp_grads, Eigen::VectorXd& grad) const {
        int num_free = getNumFreePoints();
        grad.resize(num_free * DOF);
        for (int i = 0; i < num_free; ++i) {
            int cp_idx = 3 + i;
            for (int d = 0; d < DOF; ++d) {
                grad[i * DOF + d] = cp_grads(cp_idx, d);
            }
        }
    }

    const Eigen::MatrixXd& getBasisMatrix() const { return B_; }

private:
    int K_; // Number of control points
    int M_; // Number of evaluation samples
    Eigen::MatrixXd B_; // Basis matrix (M x K)

    void precomputeBasis() {
        B_.resize(M_, K_);
        B_.setZero();

        int num_segments = K_ - 3;
        for (int m = 0; m < M_; ++m) {
            double u = (M_ > 1) ? static_cast<double>(m) / (M_ - 1) : 0.0;
            double u_scaled = u * num_segments;
            int s = std::min(static_cast<int>(std::floor(u_scaled)), num_segments - 1);
            double t = std::clamp(u_scaled - s, 0.0, 1.0);

            // Cubic B-Spline basis functions
            double b0 = (1.0 - t) * (1.0 - t) * (1.0 - t) / 6.0;
            double b1 = (3.0 * t * t * t - 6.0 * t * t + 4.0) / 6.0;
            double b2 = (-3.0 * t * t * t + 3.0 * t * t + 3.0 * t + 1.0) / 6.0;
            double b3 = (t * t * t) / 6.0;

            B_(m, s)     += b0;
            B_(m, s + 1) += b1;
            B_(m, s + 2) += b2;
            B_(m, s + 3) += b3;
        }
    }
};

} // namespace vamp_r2000ic
