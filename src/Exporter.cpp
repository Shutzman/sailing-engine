#include "../include/Exporter.hpp"
#include "../include/Grid.hpp"
#include <fstream>
#include <iostream>
#include <filesystem>
#include <cmath> // Required for std::hypot calculations

namespace SailingEngine {
    void exportRouteToJson(const std::string& filename, const std::vector<Node*>& route, Point start, Point destination, const Grid& grid) {
        std::filesystem::path filePath(filename);
        if (filePath.has_parent_path()) {
            std::filesystem::create_directories(filePath.parent_path());
        }

        std::ofstream file(filename);
        if (file.is_open()) {
            // Calculate Metrics
            double actualCost = route.empty() ? 0.0 : route.back()->gCost;
            double aerialDist = std::hypot(destination.x - start.x, destination.y - start.y);
            double routeDist = 0.0;
            
            if (!route.empty()) {
                for (size_t i = 1; i < route.size(); ++i) {
                    routeDist += std::hypot(route[i]->pos.x - route[i-1]->pos.x, 
                                            route[i]->pos.y - route[i-1]->pos.y);
                }
            }

            // Terminal "console.log" Output
            std::cout << "\n--- ROUTE STATISTICS ---\n";
            std::cout << "Actual Cost:     " << actualCost << "\n";
            std::cout << "Aerial Distance: " << aerialDist << " cells\n";
            std::cout << "Route Distance:  " << routeDist << " cells\n";
            std::cout << "------------------------\n\n";

            // Begin JSON output
            file << "{\n";
            file << "  \"width\": " << grid.getWidth() << ",\n";
            file << "  \"height\": " << grid.getHeight() << ",\n";
            file << "  \"start\": {\"x\": " << start.x << ", \"y\": " << start.y << "},\n";
            file << "  \"destination\": {\"x\": " << destination.x << ", \"y\": " << destination.y << "},\n";
            
            // Export Metrics
            file << "  \"metrics\": {\n";
            file << "    \"cost\": " << actualCost << ",\n";
            file << "    \"aerial_distance\": " << aerialDist << ",\n";
            file << "    \"route_distance\": " << routeDist << "\n";
            file << "  },\n";
            
            // Export land coordinates
            file << "  \"land\": [\n";
            bool firstLand = true;
            for (int x = 0; x < grid.getWidth(); ++x) {
                for (int y = 0; y < grid.getHeight(); ++y) {
                    if (grid.getNode(Point{x, y})->type == TerrainType::LAND) {
                        if (!firstLand) file << ",\n";
                        file << "    {\"x\": " << x << ", \"y\": " << y << "}";
                        firstLand = false;
                    }
                }
            }
            file << "\n  ],\n";

            // Export wind field (Subsampled every 4 cells to keep JSON manageable)
            file << "  \"wind_field\": [\n";
            bool firstWind = true;
            for (int x = 0; x < grid.getWidth(); x += 4) {
                for (int y = 0; y < grid.getHeight(); y += 4) {
                    Wind w = grid.getWindAt(Point{x, y});
                    if (!firstWind) file << ",\n";
                    file << "    {\"x\": " << x << ", \"y\": " << y << ", \"dir\": " << w.directionDegrees << ", \"spd\": " << w.speedKnots << "}";
                    firstWind = false;
                }
            }
            file << "\n  ],\n";

            // Export path coordinates
            file << "  \"path\": [\n";
            for (size_t i = 0; i < route.size(); ++i) {
                file << "    {\"x\": " << route[i]->pos.x << ", \"y\": " << route[i]->pos.y << "}" << (i + 1 < route.size() ? "," : "") << "\n";
            }
            file << "  ]\n";
            file << "}\n";
            
            std::cout << "[Exporter] Successfully saved route to: " << std::filesystem::absolute(filename) << std::endl;
        } else {
            std::cerr << "[Exporter Error] Could not open file: " << filename << std::endl;
        }
    }
}