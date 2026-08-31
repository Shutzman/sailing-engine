#pragma once
#include <vector>
#include <string>
#include <map>
#include "Grid.hpp"
#include "Node.hpp"
#include "Types.hpp"

namespace SailingEngine {
    namespace Visualizer {

        namespace Color {
            constexpr const char* RESET   = "\033[0m";
            constexpr const char* BLUE    = "\033[34m";
            constexpr const char* CYAN    = "\033[36m";
            constexpr const char* GREEN   = "\033[32m";
            constexpr const char* RED     = "\033[31m";
            constexpr const char* YELLOW  = "\033[33m";
            constexpr const char* BOLD    = "\033[1m";
        }

        // To add a new visual entity:
        // 1. Add the enum value here in RenderEntity.
        // 2. Register its symbol, color, and label in StyleRegistry::styles (Visualizer.cpp).
        // 3. Update StyleRegistry::determineEntity (Visualizer.cpp) with its display condition.
        enum class RenderEntity {
            WATER,
            LAND,
            RESTRICTED,
            PATH,
            START,
            DESTINATION
        };

        struct EntityStyle {
            std::string symbol;
            std::string color;
            std::string label;
        };

        class StyleRegistry {
        public:
            static const std::map<RenderEntity, EntityStyle> styles;
            
            static RenderEntity determineEntity(const Node* node, bool isStart, bool isEnd, bool isPath);
            static void printLegend();
        };

        void printGrid(const Grid& grid, const std::vector<Node*>& path, Point start, Point destination);
    } // namespace Visualizer
} // namespace SailingEngine