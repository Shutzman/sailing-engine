#include <iostream>
#include <vector>
#include <cassert> // <-- The C++ assertion library
#include <cmath>   // For std::abs

#include "../include/Grid.hpp"
#include "../include/Vessel.hpp"
#include "../include/Pathfinder.hpp"
#include "../include/Visualizer.hpp"
#include "../include/Types.hpp"
#include "../include/Exporter.hpp"

using namespace SailingEngine;

void runSailingboatScenario();
void runMotorboatScenario();
void runTrappedSailboatScenario();
void runHybridboatScenario();
void runSailingboatScenarioLargeScale();

int main() {
    std::cout << "Running Sailing Engine Test Suite...\n\n";
    
    //runSailingboatScenario();
    //runMotorboatScenario();
    //runTrappedSailboatScenario();
    //runHybridboatScenario();
    runSailingboatScenarioLargeScale();
    
    return 0;
}

void runSailingboatScenario() {
    Grid ocean(20, 10);
    ocean.setUniformWind(Wind(90.0, 15.0));
    Vessel sailboat(12.0, 3.5, 2.0, PropulsionType::SAIL_ONLY);
    Pathfinder pathfinder(ocean);
    
    Point start{2, 5};
    Point destination{17, 5};

    std::vector<Node*> route = pathfinder.findPath(start, destination, sailboat);
    
    Visualizer::printGrid(ocean, route, start, destination); // Uncomment to see visualization

    double actualCost = route.back()->gCost;

    std::cout << "\nActual cost: " << actualCost << "\n"; // Uncomment to see actual cost
}

void runMotorboatScenario() {
    Grid ocean(20, 10);
    ocean.setUniformWind(Wind(90.0, 15.0)); 
    
    Vessel motorboat(12.0, 3.5, 2.0, PropulsionType::ENGINE_ONLY);
    Pathfinder pathfinder(ocean);
    
    Point start{2, 5};
    Point destination{17, 5};

    std::vector<Node*> route = pathfinder.findPath(start, destination, motorboat);
    
    Visualizer::printGrid(ocean, route, start, destination); // Uncomment to see visualization

    double actualCost = route.back()->gCost;

    std::cout << "\nActual cost: " << actualCost << "\n"; // Uncomment to see actual cost
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
    Visualizer::printGrid(ocean, route, start, destination); // Uncomment to see visualization

}

void runHybridboatScenario() {
    Grid ocean(20, 10);
    ocean.setUniformWind(Wind(90.0, 15.0)); 
    
    // Build the exact same narrow channel
    for (int x = 4; x <= 17; ++x) {
        ocean.setTerrainType(Point{x, 4}, TerrainType::LAND);
        ocean.setTerrainType(Point{x, 6}, TerrainType::LAND);
    }
    ocean.setTerrainType(Point{4, 5}, TerrainType::RESTRICTED);

    Vessel hybrid(PropulsionType::HYBRID);
    Pathfinder pathfinder(ocean);
    
    Point start{2, 5};
    Point destination{5, 5};

    std::vector<Node*> route = pathfinder.findPath(start, destination, hybrid);
    
    Visualizer::printGrid(ocean, route, start, destination); // Uncomment to view the route!

    double actualCost = route.back()->gCost; 

    std::cout << "\nActual cost: " << actualCost << "\n"; // Uncomment to see actual cost  
}

void runSailingboatScenarioLargeScale() {
    Grid ocean(100, 80);
    ocean.setUniformWind(Wind(90.0, 10.0));
    Vessel sailboat(PropulsionType::SAIL_ONLY);
    Pathfinder pathfinder(ocean);
    
    Point start{2, 10};
    Point destination{70, 15};

    std::vector<Node*> route = pathfinder.findPath(start, destination, sailboat);
    
    // Visualizer::printGrid(ocean, route, start, destination); // Uncomment to see visualization

    if (!route.empty()) {
        double actualCost = route.back()->gCost;
        std::cout << "\nActual cost: " << actualCost << "\n";
        
        exportRouteToJson("../../../output/route_output.json", route, start, destination);
    } else {
        std::cout << "\nNo path available between start and destination.\n";
    }
}