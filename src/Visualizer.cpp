#include "../include/Visualizer.hpp"
#include <iostream>
#include <algorithm>

namespace SailingEngine {
    namespace Visualizer {

        const std::map<RenderEntity, EntityStyle> StyleRegistry::styles = {
            {RenderEntity::WATER,       {"~ ", Color::BLUE, "Water"}},
            {RenderEntity::LAND,        {"# ", Color::YELLOW, "Land"}},
            {RenderEntity::RESTRICTED,  {"X ", Color::RED, "Restricted"}},
            {RenderEntity::PATH_SAILS,  {"* ", std::string(Color::BOLD) + Color::CYAN, "Path (sails)"}},
            {RenderEntity::PATH_ENGINE, {"* ", std::string(Color::BOLD) + Color::MAGENTA, "Path (Engine)"}},
            {RenderEntity::START,       {"S ", std::string(Color::BOLD) + Color::CYAN, "Start"}},
            {RenderEntity::DESTINATION, {"D ", std::string(Color::BOLD) + Color::CYAN, "Destination"}},
        };

        RenderEntity StyleRegistry::determineEntity(const Node* node, bool isStart, bool isEnd, bool isPath) {
            if (isStart) return RenderEntity::START;
            if (isEnd) return RenderEntity::DESTINATION;
            if (isPath) return node->usedEngine ? RenderEntity::PATH_ENGINE : RenderEntity::PATH_SAILS;

            if (node->type == TerrainType::LAND) return RenderEntity::LAND;
            if (node->type == TerrainType::RESTRICTED) return RenderEntity::RESTRICTED;
            
            return RenderEntity::WATER;
        }

        void StyleRegistry::printLegend() {
            std::cout << "Legend: ";
            bool first = true;
            
            for (const auto& [entity, style] : styles) {
                if (!first) std::cout << " | ";
                std::cout << style.color << style.symbol << style.label << Color::RESET;
                first = false;
            }
            std::cout << "\n\n";
        }

        void printGrid(const Grid& grid, const std::vector<Node*>& path, Point start, Point destination) {
                    std::cout << "\n" << Color::BOLD << "--- Tactical Sailing Route ---" << Color::RESET << "\n";
                    
                    for (int y = 0; y < grid.getHeight(); ++y) {
                        for (int x = 0; x < grid.getWidth(); ++x) {
                            Point currentPos{x, y};
                            const Node* currentNode = grid.getNode(currentPos);
                            
                            // The Point operator== handles the x and y comparison for us!
                            bool isStart = (currentPos == start);
                            bool isEnd = (currentPos == destination);
                            bool isPath = (std::find(path.begin(), path.end(), currentNode) != path.end());
                            
                            RenderEntity entityType = StyleRegistry::determineEntity(currentNode, isStart, isEnd, isPath);
                            
                            const EntityStyle& style = StyleRegistry::styles.at(entityType);
                            std::cout << style.color << style.symbol << Color::RESET;
                        }
                        std::cout << "\n";
                    }
                    
                    StyleRegistry::printLegend();
                }

    } // namespace Visualizer
} // namespace SailingEngine