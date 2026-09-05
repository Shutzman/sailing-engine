#pragma once

#include <iostream>
#include <vector>
#include <cassert>
#include <cmath>

#include "../include/Grid.hpp"
#include "../include/Vessel.hpp"
#include "../include/Pathfinder.hpp"
#include "../include/Visualizer.hpp"
#include "../include/Types.hpp"

namespace SailingEngine::Tests {

    constexpr double PATH_TOLERANCE = 0.05;

    inline void runSailingboatScenario() {
        Grid ocean(20, 10);
        ocean.setUniformWind(Wind(90.0, 15.0));
        Vessel sailboat(12.0, 3.5, 2.0, PropulsionType::SAIL_ONLY);
        Pathfinder pathfinder(ocean);
        
        Point start{2, 5};
        Point destination{17, 5};

        std::vector<Node*> route = pathfinder.findPath(start, destination, sailboat);
        assert(!route.empty() && "Sailboat route should not be empty!");

        double actualCost = route.back()->gCost;
        // Verify path was found and gCost is positive
        assert(actualCost > 0.0);

        std::cout << "  [PASS] Sailing Scenario (Final Cost: " << actualCost << ")\n";
    }

    inline void runMotorboatScenario() {
        Grid ocean(20, 10);
        ocean.setUniformWind(Wind(90.0, 15.0)); 
        
        Vessel motorboat(12.0, 3.5, 2.0, PropulsionType::ENGINE_ONLY);
        Pathfinder pathfinder(ocean);
        
        Point start{2, 5};
        Point destination{17, 5};

        std::vector<Node*> route = pathfinder.findPath(start, destination, motorboat);
        assert(!route.empty() && "Motorboat route should not be empty!");

        double actualCost = route.back()->gCost;
        assert(actualCost > 0.0);

        std::cout << "  [PASS] Motorboat Scenario (Final Cost: " << actualCost << ")\n";
    }

    inline void runTrappedSailboatScenario() {
        Grid ocean(20, 10);
        ocean.setUniformWind(Wind(90.0, 15.0)); 
        
        for (int x = 4; x <= 16; ++x) {
            ocean.setTerrainType(Point{x, 4}, TerrainType::LAND);
            ocean.setTerrainType(Point{x, 6}, TerrainType::LAND);
        }
        ocean.setTerrainType(Point{16, 5}, TerrainType::LAND);

        Vessel sailboat(12.0, 3.5, 2.0, PropulsionType::SAIL_ONLY);
        Pathfinder pathfinder(ocean);
        
        Point start{2, 5};
        Point destination{15, 5};

        std::vector<Node*> route = pathfinder.findPath(start, destination, sailboat);
        assert(route.empty() && "Sailboat should be trapped without tacking room!");

        std::cout << "  [PASS] Trapped Sailboat Scenario (No route possible)\n";
    }

    inline void runHybridboatScenario() {
        Grid ocean(20, 10);
        ocean.setUniformWind(Wind(90.0, 15.0)); 
        
        for (int x = 4; x <= 17; ++x) {
            ocean.setTerrainType(Point{x, 4}, TerrainType::LAND);
            ocean.setTerrainType(Point{x, 6}, TerrainType::LAND);
        }
        ocean.setTerrainType(Point{4, 5}, TerrainType::RESTRICTED);

        Vessel hybrid(12.0, 3.5, 2.0, PropulsionType::HYBRID);
        Pathfinder pathfinder(ocean);
        
        Point start{2, 5};
        Point destination{5, 5};

        std::vector<Node*> route = pathfinder.findPath(start, destination, hybrid);
        assert(!route.empty() && "Hybrid boat route should not be empty!");

        double actualCost = route.back()->gCost;
        assert(actualCost > 0.0);

        std::cout << "  [PASS] Hybrid Scenario (Final Cost: " << actualCost << ")\n";
    }

    inline void runAllPathfinderTests() {
        std::cout << "[RUNNING] Pathfinder Integration Test Suite...\n";
        runSailingboatScenario();
        runMotorboatScenario();
        runTrappedSailboatScenario();
        runHybridboatScenario();
        std::cout << "[PASSED] Pathfinder Integration Suite Completed.\n\n";
    }

} // namespace SailingEngine::Tests