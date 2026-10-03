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
void runSailingboatScenarioMassiveScale();

int main() {
    std::cout << "Running Sailing Engine Test Suite...\n\n";
    
    //runSailingboatScenario();
    //runMotorboatScenario();
    //runTrappedSailboatScenario();
    //runHybridboatScenario();
    //runSailingboatScenarioLargeScale();
    runSailingboatScenarioMassiveScale();
    
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
        
        exportRouteToJson("route_output.json", route, start, destination, ocean);
    } else {
        std::cout << "\nNo path available between start and destination.\n";
    }
}

void runSailingboatScenarioMassiveScale() {
    // Initialize a massive 150x150 grid
    Grid ocean(150, 150);
    
    // 1. Global Wind: Strong Easterly (blowing East to West at 20 knots)
    ocean.setUniformWind(Wind(90.0, 20.0));

    // 2. Build the Start Bay (West side, opening to the East)
    for (int x = 0; x <= 25; ++x) {
        ocean.setTerrainType(Point{x, 15}, TerrainType::LAND); // North Wall
        ocean.setTerrainType(Point{x, 25}, TerrainType::LAND); // South Wall
    }
    for (int y = 15; y <= 25; ++y) {
        ocean.setTerrainType(Point{0, y}, TerrainType::LAND);  // West Wall
    }

    // 3. Build the Destination Bay (East side, opening to the West)
    for (int x = 125; x < 150; ++x) {
        ocean.setTerrainType(Point{x, 110}, TerrainType::LAND); // North Wall
        ocean.setTerrainType(Point{x, 130}, TerrainType::LAND); // South Wall
    }
    for (int y = 110; y <= 130; ++y) {
        ocean.setTerrainType(Point{149, y}, TerrainType::LAND); // East Wall
    }

    // 4. Central Blockade & Archipelago
    // Massive central island forcing a North or South routing decision
    for (int x = 60; x <= 80; ++x) {
        for (int y = 40; y <= 110; ++y) {
            ocean.setTerrainType(Point{x, y}, TerrainType::LAND);
        }
    }
    
    // Scattered archipelago near the destination bay entrance
    ocean.setTerrainType(Point{105, 115}, TerrainType::LAND);
    ocean.setTerrainType(Point{106, 115}, TerrainType::LAND);
    ocean.setTerrainType(Point{115, 125}, TerrainType::LAND);
    ocean.setTerrainType(Point{116, 126}, TerrainType::LAND);
    ocean.setTerrainType(Point{112, 105}, TerrainType::LAND);

    // 5. Local Environmental Wind Zones
    // Wind shadow (becalmed area) directly West of the massive central island
    ocean.setWindArea(Point{35, 45}, Point{59, 105}, Wind(90.0, 3.0)); 
    
    // Thermal wind shift inside the Start Bay (Offshore breeze blowing out of the bay)
    ocean.setWindArea(Point{1, 16}, Point{25, 24}, Wind(270.0, 12.0));
    
    // Swirling cross-wind at the Destination Bay entrance
    ocean.setWindArea(Point{115, 110}, Point{124, 130}, Wind(45.0, 18.0));

    // 6. Pathfinder Execution
    Vessel sailboat(PropulsionType::SAIL_ONLY);
    Pathfinder pathfinder(ocean);
    
    Point start{5, 20};       // Trapped deep inside the West bay
    Point destination{140, 120}; // Trapped deep inside the East bay

    std::vector<Node*> route = pathfinder.findPath(start, destination, sailboat);
    
    if (!route.empty()) {
        std::cout << "\n[Massive Scale] Path found! Actual cost: " << route.back()->gCost << "\n";
        exportRouteToJson("route_output.json", route, start, destination, ocean);
    } else {
        std::cout << "\n[Massive Scale] No path available between start and destination.\n";
    }
}