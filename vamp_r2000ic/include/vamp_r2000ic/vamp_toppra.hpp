#pragma once

#include <vector>
#include <memory>

namespace vamp_r2000ic {

struct TimedTrajectory {
    std::vector<std::vector<double>> positions;
    std::vector<std::vector<double>> velocities;
    std::vector<std::vector<double>> accelerations;
    std::vector<double> time_stamps;

    void clear() {
        positions.clear();
        velocities.clear();
        accelerations.clear();
        time_stamps.clear();
    }

    size_t size() const {
        return positions.size();
    }

    bool empty() const {
        return positions.empty();
    }
};

/**
 * @brief TOPP-RA Time-Optimal Path Parameterization wrapper for Fanuc R-2000iC.
 * Uses PIMPL to isolate TOPPRA and Eigen/StdVector headers from other compilation units.
 */
class VampToppra {
public:
    VampToppra();
    ~VampToppra();

    void setLimits(const std::vector<double>& vel_limits, const std::vector<double>& acc_limits);

    /**
     * @brief Computes time-optimal parameterization for given geometric waypoints.
     * @param waypoints Vector of 6-element joint waypoints.
     * @param trajectory_out Output timed trajectory with positions, velocities, accelerations, time_stamps.
     * @param max_vel_scaling Scale factor for joint velocity limits (0.01 ~ 1.0).
     * @param max_acc_scaling Scale factor for joint acceleration limits (0.01 ~ 1.0).
     * @param sample_dt Output trajectory sampling step in seconds (e.g. 0.01s = 100Hz).
     * @return true on success, false if TOPPRA fails.
     */
    bool parameterize(const std::vector<std::vector<double>>& waypoints,
                      TimedTrajectory& trajectory_out,
                      double max_vel_scaling = 1.0,
                      double max_acc_scaling = 1.0,
                      double sample_dt = 0.01);

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace vamp_r2000ic
