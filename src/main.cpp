#include <iostream>
#include <vector>
#include "Grid.hpp"
#include "Vessel.hpp"
#include "Pathfinder.hpp"
#include "Visualizer.hpp"

using namespace SailingEngine;

int main() {
    // Initialize simulation environment
    const int width = 20;
    const int height = 10;
    Grid ocean(width, height);

    // Define environmental obstacles and restricted navigation zones
    ocean.setTerrainType(9, 3, TerrainType::LAND);
    ocean.setTerrainType(9, 4, TerrainType::LAND);
    ocean.setTerrainType(9, 5, TerrainType::LAND);
    ocean.setTerrainType(9, 6, TerrainType::LAND);

    ocean.setTerrainType(12, 7, TerrainType::RESTRICTED);
    ocean.setTerrainType(13, 7, TerrainType::RESTRICTED);
    ocean.setTerrainType(12, 8, TerrainType::RESTRICTED);
    ocean.setTerrainType(13, 8, TerrainType::RESTRICTED);

    // Vessel specifications: LOA = 12.0m, Beam = 3.5m, Draft = 2.0m
    Vessel myBoat(12.0, 3.5, 2.0, PropulsionType::SAIL_ONLY);

    const int startX = 2;
    const int startY = 5;
    const int targetX = 17;
    const int targetY = 5;

    // Execute pathfinding
    Pathfinder pathfinder(ocean);
    std::vector<Node*> route = pathfinder.findPath(startX, startY, targetX, targetY, myBoat);

    if (!route.empty()) {
        std::cout << "Path found! Distance: " << route.size() << " tiles.\n";
        std::cout << "Total Distance: " << route.back()->gCost << " units (NM / meters).\n";
    } else {
        std::cout << "No path available.\n";
    }

    // Render tactical output
    Visualizer::printGrid(ocean, route, startX, startY, targetX, targetY);

    return 0;
}