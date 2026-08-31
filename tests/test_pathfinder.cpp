#include <iostream>
#include <vector>
#include <cassert> // <-- The C++ assertion library
#include <cmath>   // For std::abs

#include "../include/Grid.hpp"
#include "../include/Vessel.hpp"
#include "../include/Pathfinder.hpp"
#include "../include/Visualizer.hpp"
#include "../include/Types.hpp"

using namespace SailingEngine;

void runSailingboatScenario();
void runMotorboatScenario();
void runTrappedSailboatScenario();
void runHybridRescueScenario();

int main() {
    std::cout << "Running Sailing Engine Test Suite...\n\n";
    
    runSailingboatScenario();
    runMotorboatScenario();
    runTrappedSailboatScenario();
    runHybridRescueScenario();
    
    std::cout << "\nAll tests passed successfully!\n";
    return 0;
}

void runSailingboatScenario() {
    Grid ocean(20, 10);
    ocean.setUniformWind(Wind(90.0, 15.0));
    Vessel sailboat(12.0, 3.5, 2.0, PropulsionType::SAIL_ONLY);
    Pathfinder pathfinder(ocean);
    
    Point start{2, 5};
    Point target{17, 5};

    std::vector<Node*> route = pathfinder.findPath(start, target, sailboat);
    // Visualizer::printGrid(ocean, route, start, target); // Uncomment to see visualization

    assert(!route.empty() && "Test Failed: Route should not be empty!");

    double expectedCost = 32.615; 
    double actualCost = route.back()->gCost;
    bool costMatches = std::abs(actualCost - expectedCost) < 0.01;

    assert(costMatches && "Test Failed: Path cost did not match expected cost!");

    std::cout << "[PASS] Tactical Sailing Scenario\n";
}

void runMotorboatScenario() {
    Grid ocean(20, 10);
    ocean.setUniformWind(Wind(90.0, 15.0)); 
    
    Vessel motorboat(12.0, 3.5, 2.0, PropulsionType::ENGINE_ONLY);
    Pathfinder pathfinder(ocean);
    
    Point start{2, 5};
    Point destination{17, 5};

    std::vector<Node*> route = pathfinder.findPath(start, destination, motorboat);
    // Visualizer::printGrid(ocean, route, start, target); // Uncomment to see visualization

    assert(!route.empty() && "Test Failed: Route should not be empty!");

    double expectedCost = 30.0; 
    double actualCost = route.back()->gCost;
    bool costMatches = std::abs(actualCost - expectedCost) < 0.01;

    assert(costMatches && "Test Failed: Path cost did not match expected cost!");

    std::cout << "[PASS] Motorboat Scenario\n";
}

void runTrappedSailboatScenario() {
    Grid ocean(20, 10);
    // 90-degree wind creates a dead headwind against Eastward travel
    ocean.setUniformWind(Wind(90.0, 15.0)); 
    
    // Build a narrow horizontal channel (walls at Y=4 and Y=6)
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
    // Visualizer::printGrid(ocean, route, start, target); // Uncomment to see visualization

    // Assert that the route is EMPTY because the sailboat cannot tack in the narrow channel!
    assert(route.empty() && "Test Failed: Sailboat should be trapped!");
    
    std::cout << "[PASS] Trapped Sailboat Scenario (No route possible)\n";
}

void runHybridRescueScenario() {
    Grid ocean(20, 10);
    ocean.setUniformWind(Wind(90.0, 15.0)); 
    
    // Build the exact same narrow channel
    for (int x = 4; x <= 17; ++x) {
        ocean.setTerrainType(Point{x, 4}, TerrainType::LAND);
        ocean.setTerrainType(Point{x, 6}, TerrainType::LAND);
    }
    ocean.setTerrainType(Point{4, 5}, TerrainType::RESTRICTED);

    Vessel hybrid(12.0, 3.5, 2.0, PropulsionType::HYBRID);
    Pathfinder pathfinder(ocean);
    
    Point start{2, 5};
    Point target{5, 5};

    std::vector<Node*> route = pathfinder.findPath(start, target, hybrid);
    // Visualizer::printGrid(ocean, route, start, target); // Uncomment to view the route!

    
    // Assert that the route is NOT empty
    assert(!route.empty() && "Test Failed: Hybrid should have powered through!");
    
    // Distance is 15 straight tiles. 
    // 15 tiles * 1.0 base cost * 2.0 headwind engine penalty = 30.0
    double expectedCost = 45.777; 
    double actualCost = route.back()->gCost; 
    bool costMatches = std::abs(actualCost - expectedCost) < 0.01;

    assert(costMatches && "Test Failed: Hybrid cost mismatch!");
    
    std::cout << "[PASS] Hybrid Rescue Scenario\n";
}