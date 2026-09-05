#pragma once

#include "r2000ic_collision.hpp"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/MotionValidator.h>

namespace vamp_r2000ic {

/**
 * @brief OMPL State Space configured with Fanuc R-2000iC/165F joint limits.
 */
inline std::shared_ptr<ompl::base::RealVectorStateSpace> createR2000icStateSpace() {
    auto space = std::make_shared<ompl::base::RealVectorStateSpace>(6);

    ompl::base::RealVectorBounds bounds(6);
    // J1: [-185, 185] deg
    bounds.setLow(0, -3.228859);
    bounds.setHigh(0, 3.228859);
    // J2: [-60, 76] deg
    bounds.setLow(1, -1.047197);
    bounds.setHigh(1, 1.326450);
    // J3: [-79, 180] deg
    bounds.setLow(2, -1.378810);
    bounds.setHigh(2, 3.141592);
    // J4: [-360, 360] deg
    bounds.setLow(3, -6.283185);
    bounds.setHigh(3, 6.283185);
    // J5: [-125, 125] deg
    bounds.setLow(4, -2.181661);
    bounds.setHigh(4, 2.181661);
    // J6: [-360, 360] deg
    bounds.setLow(5, -6.283185);
    bounds.setHigh(5, 6.283185);

    space->setBounds(bounds);
    return space;
}

/**
 * @brief OMPL State Validity Checker using SIMD CollisionChecker
 */
class VampStateValidityChecker : public ompl::base::StateValidityChecker {
public:
    VampStateValidityChecker(const ompl::base::SpaceInformationPtr& si,
                             const CollisionChecker& checker,
                             float safety_margin = 0.025f)
        : ompl::base::StateValidityChecker(si)
        , checker_(checker)
        , safety_margin_(safety_margin) {}

    bool isValid(const ompl::base::State* state) const override {
        const auto* rstate = state->as<ompl::base::RealVectorStateSpace::StateType>();
        double joints[6] = {
            rstate->values[0],
            rstate->values[1],
            rstate->values[2],
            rstate->values[3],
            rstate->values[4],
            rstate->values[5]
        };
        // isValid returns true if NO collision
        return !checker_.checkCollision(joints, safety_margin_);
    }

private:
    const CollisionChecker& checker_;
    float safety_margin_;
};

/**
 * @brief OMPL Motion Validator with vectorized discrete interpolation checking
 */
class VampMotionValidator : public ompl::base::MotionValidator {
public:
    VampMotionValidator(const ompl::base::SpaceInformationPtr& si,
                        const CollisionChecker& checker,
                        float safety_margin = 0.025f,
                        double resolution = 0.02)
        : ompl::base::MotionValidator(si)
        , checker_(checker)
        , safety_margin_(safety_margin)
        , resolution_(resolution) {}

    bool checkMotion(const ompl::base::State* s1, const ompl::base::State* s2) const override {
        std::pair<ompl::base::State*, double> unused;
        return checkMotion(s1, s2, unused);
    }

    bool checkMotion(const ompl::base::State* s1, const ompl::base::State* s2,
                     std::pair<ompl::base::State*, double>& lastValid) const override {
        const auto* r1 = s1->as<ompl::base::RealVectorStateSpace::StateType>();
        const auto* r2 = s2->as<ompl::base::RealVectorStateSpace::StateType>();

        double max_dist = 0.0;
        for (int i = 0; i < 6; ++i) {
            max_dist = std::max(max_dist, std::abs(r2->values[i] - r1->values[i]));
        }

        int steps = std::max(1, static_cast<int>(std::ceil(max_dist / resolution_)));
        double joints[6];

        for (int step = 0; step <= steps; ++step) {
            double alpha = static_cast<double>(step) / steps;
            for (int i = 0; i < 6; ++i) {
                joints[i] = (1.0 - alpha) * r1->values[i] + alpha * r2->values[i];
            }

            if (checker_.checkCollision(joints, safety_margin_)) {
                return false;
            }
        }
        return true;
    }

private:
    const CollisionChecker& checker_;
    float safety_margin_;
    double resolution_;
};

} // namespace vamp_r2000ic
