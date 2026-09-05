#include "../include/NauticalCostModel.hpp"
#include <cmath>
#include <algorithm>

namespace SailingEngine {

    double NauticalCostModel::calculateAngleDifference(double a, double b) {
        double diff = std::fmod(std::abs(a - b), 360.0);
        return (diff > 180.0) ? 360.0 - diff : diff;
    }

    std::optional<StepCostResult> NauticalCostModel::calculateWindEffect(
        double headingDegrees, 
        const Wind& wind, 
        PropulsionType propulsion
    ) const {
        double speed = wind.speedKnots;
        double twa = calculateAngleDifference(headingDegrees, wind.directionDegrees);

        StepCostResult result;
        result.costMultiplier = 1.0;
        result.engineActive = (propulsion == PropulsionType::ENGINE_ONLY);

        // 1. Becalmed conditions (dead wind)
        if (speed < SPEED_BECALMED) {
            if (propulsion == PropulsionType::SAIL_ONLY) {
                return std::nullopt; // Dead stop without wind
            }
            result.engineActive = true;
            result.costMultiplier = 1.0;
            return result;
        }

        // 2. Pure Sailboat Dynamics
        if (propulsion == PropulsionType::SAIL_ONLY) {
            if (twa < TWA_NO_GO) {
                return std::nullopt; // In irons - strictly impossible
            }
            if (twa < TWA_CLOSE_HAULED) {
                result.costMultiplier = 1.5; // Drag and heavy heeling penalty
            } else if (twa < TWA_BEAM_REACH) {
                result.costMultiplier = (speed >= SPEED_OPTIMAL && speed < SPEED_STORM) ? 0.6 : 0.8;
            } else {
                result.costMultiplier = (speed >= SPEED_OPTIMAL) ? 0.8 : 1.0;
            }
            result.engineActive = false;
            return result;
        }

        // 3. Pure Motorboat Dynamics
        if (propulsion == PropulsionType::ENGINE_ONLY) {
            result.engineActive = true;
            if (twa < TWA_NO_GO) {
                result.costMultiplier = 1.0 + (speed / 10.0); // Pushing directly into headseas/headwind
            } else if (twa < TWA_CLOSE_HAULED) {
                result.costMultiplier = 1.0 + (speed / 15.0);
            } else if (twa < TWA_BEAM_REACH) {
                result.costMultiplier = (speed >= SPEED_HEAVY_WEATHER) ? 1.2 : 1.0;
            } else {
                result.costMultiplier = 1.0;
            }
            return result;
        }

        // 4. Hybrid Dynamics (Sails preferred, engine auxiliary)
        if (twa < TWA_NO_GO) {
            result.engineActive = true;
            result.costMultiplier = 1.0 + (speed / 10.0); // In irons: motor engages
        } else if (twa < TWA_CLOSE_HAULED) {
            result.engineActive = false;
            result.costMultiplier = 1.5;
        } else if (twa < TWA_BEAM_REACH) {
            result.engineActive = false;
            result.costMultiplier = (speed >= SPEED_OPTIMAL && speed < SPEED_STORM) ? 0.6 : 0.8;
        } else {
            result.engineActive = false;
            result.costMultiplier = (speed >= SPEED_OPTIMAL) ? 0.8 : 1.0;
        }

        return result;
    }

    std::optional<StepCostResult> NauticalCostModel::calculateStepCost(
        double headingDegrees, 
        const Wind& wind, 
        const Vessel& vessel
    ) const {
        // Step 1: Evaluate Wind
        auto stepResult = calculateWindEffect(headingDegrees, wind, vessel.getPropulsion());
        if (!stepResult.has_value()) {
            return std::nullopt; // Hard stop: course impossible in current wind
        }

        // Step 2: Future hooks (Currents, Waves) plug in directly here:
        // double currentMultiplier = calculateCurrentEffect(...);
        // stepResult->costMultiplier *= currentMultiplier;

        return stepResult;
    }

} // namespace SailingEngine