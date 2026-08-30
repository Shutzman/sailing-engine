#include <iostream>
#include <vector>
#include "Grid.hpp"
#include "Vessel.hpp"
#include "Pathfinder.hpp"
#include "Visualizer.hpp"
#include "Environment.hpp"

using namespace SailingEngine;

int main() {
    const int width = 20;
    const int height = 10;
    Grid ocean(width, height);

    // 1. Set a global dead headwind from the East (90 degrees)
    ocean.setUniformWind(Wind(90.0, 15.0));

    // 2. Create a localized wind pocket in the middle of the map!
    // Between X=5 and X=10, the wind blows from the South (180 degrees) at 5 knots.
    ocean.setWindArea(5, 10, 0, 9, Wind(180.0, 5.0));

    // Obstacles
    ocean.setTerrainType(9, 3, TerrainType::LAND);
    ocean.setTerrainType(9, 4, TerrainType::LAND);
    ocean.setTerrainType(9, 5, TerrainType::LAND);
    ocean.setTerrainType(9, 6, TerrainType::LAND);
    ocean.setTerrainType(12, 7, TerrainType::RESTRICTED);
    ocean.setTerrainType(13, 7, TerrainType::RESTRICTED);
    ocean.setTerrainType(12, 8, TerrainType::RESTRICTED);
    ocean.setTerrainType(13, 8, TerrainType::RESTRICTED);

    Vessel myBoat(12.0, 3.5, 2.0, PropulsionType::SAIL_ONLY);

    const int startX = 2;
    const int startY = 5;
    const int targetX = 17;
    const int targetY = 5;

    Pathfinder pathfinder(ocean);
    std::vector<Node*> route = pathfinder.findPath(startX, startY, targetX, targetY, myBoat);

    if (!route.empty()) {
        std::cout << "Path found! Steps: " << route.size() << " tiles.\n";
        std::cout << "Total Traversal Cost: " << route.back()->gCost << "\n";
    } else {
        std::cout << "No path available.\n";
    }

    Visualizer::printGrid(ocean, route, startX, startY, targetX, targetY);

    return 0;
}