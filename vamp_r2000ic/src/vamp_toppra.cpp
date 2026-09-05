#include "vamp_r2000ic/vamp_toppra.hpp"

#include <cmath>
#include <algorithm>
#include <iostream>

#include <toppra/toppra.hpp>
#include <toppra/algorithm/toppra.hpp>
#include <toppra/geometric_path/piecewise_poly_path.hpp>
#include <toppra/constraint/linear_joint_velocity.hpp>
#include <toppra/constraint/linear_joint_acceleration.hpp>
#include <toppra/solver/seidel.hpp>
#include <toppra/parametrizer/const_accel.hpp>

namespace vamp_r2000ic {

struct VampToppra::Impl {
    std::vector<double> vel_limits_{ 2.2689, 2.0071, 2.1817, 3.8397, 3.8397, 5.5851 };
    std::vector<double> acc_limits_{ 6.0, 5.0, 6.0, 10.0, 10.0, 15.0 };
};

VampToppra::VampToppra() : pimpl_(std::make_unique<Impl>()) {}
VampToppra::~VampToppra() = default;

void VampToppra::setLimits(const std::vector<double>& vel_limits, const std::vector<double>& acc_limits) {
    if (vel_limits.size() == 6) pimpl_->vel_limits_ = vel_limits;
    if (acc_limits.size() == 6) pimpl_->acc_limits_ = acc_limits;
}

bool VampToppra::parameterize(const std::vector<std::vector<double>>& waypoints,
                              TimedTrajectory& trajectory_out,
                              double max_vel_scaling,
                              double max_acc_scaling,
                              double sample_dt) {
    trajectory_out.clear();
    int M = static_cast<int>(waypoints.size());
    if (M < 3) return false;

    max_vel_scaling = std::clamp(max_vel_scaling, 0.01, 1.0);
    max_acc_scaling = std::clamp(max_acc_scaling, 0.01, 1.0);

    // Convert waypoints into toppra Vectors
    toppra::Vectors positions(M);
    for (int i = 0; i < M; ++i) {
        positions[i].resize(6);
        for (int d = 0; d < 6; ++d) {
            positions[i][d] = waypoints[i][d];
        }
    }

    toppra::Vector times(M);
    times[0] = 0.0;
    for (int i = 1; i < M; ++i) {
        double dist = 0.0;
        for (int d = 0; d < 6; ++d) {
            double diff = waypoints[i][d] - waypoints[i - 1][d];
            dist += diff * diff;
        }
        times[i] = times[i - 1] + std::max(1e-4, std::sqrt(dist));
    }

    // Clamped boundary condition: zero velocity at start and end
    toppra::BoundaryCond bc{1, std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    toppra::BoundaryCondFull bc_type{bc, bc};

    auto path = std::make_shared<toppra::PiecewisePolyPath>(
        toppra::PiecewisePolyPath::CubicSpline(positions, times, bc_type));

    // Formulate constraints with safety margin against discrete spline sampling overshoot
    toppra::Vector v_lim(6), a_lim(6);
    for (int d = 0; d < 6; ++d) {
        v_lim[d] = pimpl_->vel_limits_[d] * max_vel_scaling * 0.95;
        a_lim[d] = pimpl_->acc_limits_[d] * max_acc_scaling * 0.90;
    }

    toppra::LinearConstraintPtrs constraints{
        std::make_shared<toppra::constraint::LinearJointVelocity>(-v_lim, v_lim),
        std::make_shared<toppra::constraint::LinearJointAcceleration>(-a_lim, a_lim)
    };

    for (auto& c : constraints) {
        c->discretizationType(toppra::DiscretizationType::Collocation);
    }

    // Initialize and solve TOPPRA problem with dense collocation grid
    toppra::algorithm::TOPPRA problem(constraints, path);
    problem.setN(std::max(M * 2, 100));
    problem.solver(std::make_shared<toppra::solver::Seidel>());

    auto ret_code = problem.computePathParametrization();
    if (ret_code != toppra::ReturnCode::OK) {
        return false;
    }

    const auto& data = problem.getParameterizationData();
    toppra::parametrizer::ConstAccel traj(path, data.gridpoints, data.parametrization);

    auto interval = traj.pathInterval();
    double total_duration = interval(1);
    if (total_duration <= 0.0) return false;

    // Sample at fixed dt
    if (sample_dt <= 0.0) sample_dt = 0.01;
    int num_steps = static_cast<int>(std::ceil(total_duration / sample_dt)) + 1;

    trajectory_out.positions.reserve(num_steps);
    trajectory_out.velocities.reserve(num_steps);
    trajectory_out.accelerations.reserve(num_steps);
    trajectory_out.time_stamps.reserve(num_steps);

    for (int i = 0; i < num_steps; ++i) {
        double t = std::min(i * sample_dt, total_duration);

        toppra::Vector q = traj.eval_single(t, 0);
        toppra::Vector qd = traj.eval_single(t, 1);
        toppra::Vector qdd = traj.eval_single(t, 2);

        std::vector<double> pos(6), vel(6), acc(6);
        for (int d = 0; d < 6; ++d) {
            double v_limit = pimpl_->vel_limits_[d] * max_vel_scaling;
            double a_limit = pimpl_->acc_limits_[d] * max_acc_scaling;
            pos[d] = q[d];
            vel[d] = std::clamp(qd[d], -v_limit, v_limit);
            acc[d] = std::clamp(qdd[d], -a_limit, a_limit);
        }

        if (i == 0 || i == num_steps - 1) {
            std::fill(vel.begin(), vel.end(), 0.0);
        }

        trajectory_out.positions.push_back(std::move(pos));
        trajectory_out.velocities.push_back(std::move(vel));
        trajectory_out.accelerations.push_back(std::move(acc));
        trajectory_out.time_stamps.push_back(t);

        if (t >= total_duration) break;
    }

    return true;
}

} // namespace vamp_r2000ic
