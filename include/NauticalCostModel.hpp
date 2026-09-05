#pragma once

#include <optional>
#include "Vessel.hpp"
#include "Environment.hpp"

namespace SailingEngine {

    struct StepCostResult {
        double costMultiplier = 1.0;
        bool engineActive = false;
    };

    class NauticalCostModel {
    public:
        NauticalCostModel() = default;

        // Pipeline coordinator: evaluates all environmental forces for one step
        std::optional<StepCostResult> calculateStepCost(
            double headingDegrees, 
            const Wind& wind, 
            const Vessel& vessel
        ) const;

        // Utility: Computes the shortest angular difference between two bearings [0.0, 180.0]
        static double calculateAngleDifference(double a, double b);

    private:
        // Dedicated environmental calculator
        std::optional<StepCostResult> calculateWindEffect(
            double headingDegrees, 
            const Wind& wind, 
            PropulsionType propulsion
        ) const;

        // Threshold constants
        static constexpr double TWA_NO_GO = 45.0;
        static constexpr double TWA_CLOSE_HAULED = 60.0;
        static constexpr double TWA_BEAM_REACH = 120.0;

        static constexpr double SPEED_BECALMED = 3.0;
        static constexpr double SPEED_OPTIMAL = 20.0;
        static constexpr double SPEED_HEAVY_WEATHER = 25.0;
        static constexpr double SPEED_STORM = 45.0;
    };

} // namespace SailingEngine