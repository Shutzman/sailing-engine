#pragma once

#include <iostream>
#include <cassert>
#include <cmath>
#include "../include/NauticalCostModel.hpp"
#include "../include/Vessel.hpp"
#include "../include/Environment.hpp"

namespace SailingEngine::Tests {

    constexpr double PHYSICS_TOLERANCE = 0.001;

    inline void testAngleNormalization() {
        assert(std::abs(NauticalCostModel::calculateAngleDifference(350.0, 10.0) - 20.0) < PHYSICS_TOLERANCE);
        assert(std::abs(NauticalCostModel::calculateAngleDifference(10.0, 350.0) - 20.0) < PHYSICS_TOLERANCE);
        assert(std::abs(NauticalCostModel::calculateAngleDifference(0.0, 180.0) - 180.0) < PHYSICS_TOLERANCE);
        std::cout << "  [PASS] Angle Difference Normalization\n";
    }

    inline void testSailboatInIrons() {
        NauticalCostModel model;
        Vessel sailboat(12.0, 3.5, 2.0, PropulsionType::SAIL_ONLY);
        Wind headwind(0.0, 15.0); // Wind from North (0 deg)

        // Heading 0 deg is directly into the wind (TWA = 0 < 45) -> In Irons
        auto result = model.calculateStepCost(0.0, headwind, sailboat);
        assert(!result.has_value() && "Sailboat should be blocked in irons!");

        // Heading 30 deg is still in no-go zone (< 45)
        auto result30 = model.calculateStepCost(30.0, headwind, sailboat);
        assert(!result30.has_value() && "Sailboat should be blocked at 30 deg TWA!");

        std::cout << "  [PASS] Sailboat In Irons (No-Go Zone)\n";
    }

    inline void testMotorboatInHeadwind() {
        NauticalCostModel model;
        Vessel motorboat(12.0, 3.5, 2.0, PropulsionType::ENGINE_ONLY);
        Wind headwind(0.0, 20.0);

        auto result = model.calculateStepCost(0.0, headwind, motorboat);
        assert(result.has_value() && "Motorboat must be able to push through headwind!");
        assert(result->engineActive == true);
        // Cost: 1.0 + (20 / 10.0) = 3.0
        assert(std::abs(result->costMultiplier - 3.0) < PHYSICS_TOLERANCE);

        std::cout << "  [PASS] Motorboat Headwind Resistance\n";
    }

    inline void testBecalmedConditions() {
        NauticalCostModel model;
        Vessel sailboat(12.0, 3.5, 2.0, PropulsionType::SAIL_ONLY);
        Vessel hybrid(12.0, 3.5, 2.0, PropulsionType::HYBRID);
        Wind calmWind(90.0, 1.5); // < 3 knots

        auto sailResult = model.calculateStepCost(90.0, calmWind, sailboat);
        assert(!sailResult.has_value() && "Sailboat cannot move when becalmed!");

        auto hybridResult = model.calculateStepCost(90.0, calmWind, hybrid);
        assert(hybridResult.has_value() && "Hybrid boat must engage engine when becalmed!");
        assert(hybridResult->engineActive == true);

        std::cout << "  [PASS] Becalmed Environmental Handling\n";
    }

    inline void runAllNauticalCostModelTests() {
        std::cout << "[RUNNING] NauticalCostModel Test Suite...\n";
        testAngleNormalization();
        testSailboatInIrons();
        testMotorboatInHeadwind();
        testBecalmedConditions();
        std::cout << "[PASSED] NauticalCostModel Suite Completed.\n\n";
    }

} // namespace SailingEngine::Tests