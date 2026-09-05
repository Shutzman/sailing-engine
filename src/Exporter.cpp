#include "../include/Exporter.hpp"
#include <fstream>
#include <iostream>
#include <filesystem>

namespace SailingEngine {
    void exportRouteToJson(const std::string& filename, const std::vector<Node*>& route, Point start, Point destination) {
        std::filesystem::path filePath(filename);
        if (filePath.has_parent_path()) {
            std::filesystem::create_directories(filePath.parent_path());
        }

        std::ofstream file(filename);
        if (file.is_open()) {
            file << "{\n";
            file << "  \"start\": {\"x\": " << start.x << ", \"y\": " << start.y << "},\n";
            file << "  \"destination\": {\"x\": " << destination.x << ", \"y\": " << destination.y << "},\n";
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