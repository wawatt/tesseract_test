#include "planner_impl.h"

namespace robot_planner {

bool RobotPlanner::computeFK(const std::vector<double>& joint_angles, std::vector<double>& pose_out) {
    if (!pimpl_->env_) {
        pimpl_->setLastError(PlannerStatus::NOT_INITIALIZED, "Environment not initialized.");
        return false;
    }
    auto joint_group = pimpl_->env_->getJointGroup(pimpl_->manipulator_name_);
    if (!joint_group) {
        pimpl_->setLastError(PlannerStatus::INTERNAL_ERROR, "No joint group found for manipulator.");
        return false;
    }
    
    if (joint_angles.size() != joint_group->getJointNames().size()) {
        pimpl_->setLastError(PlannerStatus::INVALID_ARGUMENTS, "Joint angles size mismatch.");
        return false;
    }

    Eigen::VectorXd joints = Eigen::Map<const Eigen::VectorXd>(joint_angles.data(), joint_angles.size());
    auto transforms = joint_group->calcFwdKin(joints);
    
    if (transforms.find(pimpl_->tool_link_) == transforms.end()) {
        pimpl_->setLastError(PlannerStatus::INTERNAL_ERROR, "Tool link '" + pimpl_->tool_link_ + "' not found in FK transforms.");
        return false;
    }
    Eigen::Isometry3d pose = transforms.at(pimpl_->tool_link_);
    
    pose_out.resize(7);
    pose_out[0] = pose.translation().x();
    pose_out[1] = pose.translation().y();
    pose_out[2] = pose.translation().z();
    Eigen::Quaterniond q(pose.rotation());
    pose_out[3] = q.x();
    pose_out[4] = q.y();
    pose_out[5] = q.z();
    pose_out[6] = q.w();
    
    pimpl_->setLastError(PlannerStatus::SUCCESS, "");
    return true;
}

bool RobotPlanner::computeFKForLink(const std::vector<double>& joint_angles, const std::string& link_name, std::vector<double>& pose_out) {
    if (!pimpl_->env_) {
        pimpl_->setLastError(PlannerStatus::NOT_INITIALIZED, "Environment not initialized.");
        return false;
    }
    auto joint_group = pimpl_->env_->getJointGroup(pimpl_->manipulator_name_);
    if (!joint_group) {
        pimpl_->setLastError(PlannerStatus::INTERNAL_ERROR, "No joint group found for manipulator.");
        return false;
    }
    if (joint_angles.size() != joint_group->getJointNames().size()) {
        pimpl_->setLastError(PlannerStatus::INVALID_ARGUMENTS, "Joint angles size mismatch.");
        return false;
    }

    Eigen::VectorXd joints = Eigen::Map<const Eigen::VectorXd>(joint_angles.data(), joint_angles.size());
    auto transforms = joint_group->calcFwdKin(joints);
    
    std::string target_link = link_name.empty() ? pimpl_->tool_link_ : link_name;
    if (transforms.find(target_link) == transforms.end()) {
        pimpl_->setLastError(PlannerStatus::INTERNAL_ERROR, "Link '" + target_link + "' not found in FK transforms.");
        return false;
    }
    Eigen::Isometry3d pose = transforms.at(target_link);
    
    pose_out.resize(7);
    pose_out[0] = pose.translation().x();
    pose_out[1] = pose.translation().y();
    pose_out[2] = pose.translation().z();
    Eigen::Quaterniond q(pose.rotation());
    pose_out[3] = q.x();
    pose_out[4] = q.y();
    pose_out[5] = q.z();
    pose_out[6] = q.w();
    
    pimpl_->setLastError(PlannerStatus::SUCCESS, "");
    return true;
}

bool RobotPlanner::calcJacobian(const std::vector<double>& joint_angles, std::vector<double>& jacobian_out, const std::string& link_name) {
    jacobian_out.clear();
    if (!pimpl_->env_) {
        pimpl_->setLastError(PlannerStatus::NOT_INITIALIZED, "Environment not initialized.");
        return false;
    }
    auto joint_group = pimpl_->env_->getJointGroup(pimpl_->manipulator_name_);
    if (!joint_group) {
        pimpl_->setLastError(PlannerStatus::INTERNAL_ERROR, "No joint group found for manipulator.");
        return false;
    }
    if (joint_angles.size() != joint_group->getJointNames().size()) {
        pimpl_->setLastError(PlannerStatus::INVALID_ARGUMENTS, "joint_angles size mismatch.");
        return false;
    }

    Eigen::VectorXd joints = Eigen::Map<const Eigen::VectorXd>(joint_angles.data(), joint_angles.size());
    std::string target_link = link_name.empty() ? pimpl_->tool_link_ : link_name;

    try {
        Eigen::MatrixXd J = joint_group->calcJacobian(joints, target_link);
        size_t rows = J.rows();
        size_t cols = J.cols();
        jacobian_out.resize(rows * cols);
        for (size_t r = 0; r < rows; ++r) {
            for (size_t c = 0; c < cols; ++c) {
                jacobian_out[r * cols + c] = J(r, c);
            }
        }
        pimpl_->setLastError(PlannerStatus::SUCCESS, "");
        return true;
    } catch (const std::exception& e) {
        pimpl_->setLastError(PlannerStatus::INTERNAL_ERROR, std::string("calcJacobian failed: ") + e.what());
        return false;
    }
}

bool RobotPlanner::computeManipulability(const std::vector<double>& joint_angles, double& score_out) {
    score_out = 0.0;
    std::vector<double> J_flat;
    if (!calcJacobian(joint_angles, J_flat)) {
        return false;
    }
    int cols = static_cast<int>(joint_angles.size());
    int rows = 6;
    Eigen::MatrixXd J(rows, cols);
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            J(r, c) = J_flat[r * cols + c];
        }
    }
    Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();
    double det = JJt.determinant();
    score_out = (det > 0.0) ? std::sqrt(det) : 0.0;
    pimpl_->setLastError(PlannerStatus::SUCCESS, "");
    return true;
}

bool RobotPlanner::computeAllIK(const std::vector<double>& pose, std::vector<std::vector<double>>& all_solutions_out) {
    all_solutions_out.clear();
    if (pose.size() < 7) {
        pimpl_->setLastError(PlannerStatus::INVALID_ARGUMENTS, "Target pose size must be at least 7.");
        return false;
    }

    std::vector<std::vector<double>> raw_solutions;
    if (!OpwKinematics::computeAllIK(pimpl_->opw_params_, pose, raw_solutions)) {
        pimpl_->setLastError(PlannerStatus::IK_FAILED, "No mathematical analytical inverse kinematics solution exists for pose.");
        return false;
    }

    // Filter against URDF physical joint limits (with 2*pi periodicity normalization)
    for (auto sol : raw_solutions) {
        bool in_limits = true;
        if (sol.size() == pimpl_->joint_limits_min_.size()) {
            for (size_t j = 0; j < sol.size(); ++j) {
                double min_limit = pimpl_->joint_limits_min_[j] - 1e-4;
                double max_limit = pimpl_->joint_limits_max_[j] + 1e-4;
                while (sol[j] < min_limit && (sol[j] + 2.0 * M_PI) <= max_limit) {
                    sol[j] += 2.0 * M_PI;
                }
                while (sol[j] > max_limit && (sol[j] - 2.0 * M_PI) >= min_limit) {
                    sol[j] -= 2.0 * M_PI;
                }
                if (sol[j] < min_limit || sol[j] > max_limit) {
                    in_limits = false;
                    break;
                }
            }
        }
        if (in_limits) {
            all_solutions_out.push_back(sol);
        }
    }

    if (all_solutions_out.empty()) {
        pimpl_->setLastError(PlannerStatus::JOINT_LIMIT_VIOLATED, "Analytical IK solutions exist but all violate URDF physical joint limits.");
        return false;
    }

    pimpl_->setLastError(PlannerStatus::SUCCESS, "");
    return true;
}

bool RobotPlanner::computeIK(const std::vector<double>& pose, const std::vector<double>& seed_joint_angles, std::vector<double>& joint_angles_out) {
    if (pose.size() < 7) {
        pimpl_->setLastError(PlannerStatus::INVALID_ARGUMENTS, "Target pose size must be at least 7.");
        return false;
    }

    // 1. Try OPW analytical inverse kinematics with limits check
    std::vector<std::vector<double>> valid_solutions;
    if (computeAllIK(pose, valid_solutions) && !valid_solutions.empty()) {
        double min_dist_sq = std::numeric_limits<double>::max();
        size_t best_idx = 0;
        for (size_t i = 0; i < valid_solutions.size(); ++i) {
            double dist_sq = 0.0;
            for (size_t j = 0; j < valid_solutions[i].size(); ++j) {
                double diff = valid_solutions[i][j] - (j < seed_joint_angles.size() ? seed_joint_angles[j] : 0.0);
                dist_sq += diff * diff;
            }
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                best_idx = i;
            }
        }
        joint_angles_out = valid_solutions[best_idx];
        pimpl_->setLastError(PlannerStatus::SUCCESS, "");
        return true;
    }

    // 2. Fallback to Tesseract numerical kinematics solver
    if (!pimpl_->env_) {
        pimpl_->setLastError(PlannerStatus::NOT_INITIALIZED, "Environment not initialized.");
        return false;
    }
    try {
        auto kin_group = pimpl_->env_->getKinematicGroup(pimpl_->manipulator_name_);
        if (!kin_group) {
            pimpl_->setLastError(PlannerStatus::INTERNAL_ERROR, "No kinematic group found for " + pimpl_->manipulator_name_);
            return false;
        }
        
        Eigen::Isometry3d target_pose = Eigen::Isometry3d::Identity();
        target_pose.translation() = Eigen::Vector3d(pose[0], pose[1], pose[2]);
        Eigen::Quaterniond q(pose[6], pose[3], pose[4], pose[5]);
        target_pose.linear() = q.matrix();
        
        Eigen::VectorXd seed = Eigen::Map<const Eigen::VectorXd>(seed_joint_angles.data(), seed_joint_angles.size());
        tesseract::kinematics::KinGroupIKInput ik_input(target_pose, pimpl_->base_link_, pimpl_->tool_link_);
        tesseract::kinematics::IKSolutions solutions = kin_group->calcInvKin(ik_input, seed);
        
        if (solutions.empty()) {
            pimpl_->setLastError(PlannerStatus::IK_FAILED, "Numerical IK found no solution.");
            return false;
        }
        
        Eigen::VectorXd solution = solutions[0];
        joint_angles_out.resize(solution.size());
        for (int i = 0; i < solution.size(); ++i) {
            joint_angles_out[i] = solution(i);
        }
        pimpl_->setLastError(PlannerStatus::SUCCESS, "");
        return true;
    } catch (const std::exception& e) {
        pimpl_->setLastError(PlannerStatus::INTERNAL_ERROR, std::string("computeIK exception: ") + e.what());
        return false;
    }
}

} // namespace robot_planner
