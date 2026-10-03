#pragma once
#include <vector>
#include <string>
#include "Node.hpp"
#include "Types.hpp"
#include "Grid.hpp"

namespace SailingEngine {
    void exportRouteToJson(const std::string& filename, const std::vector<Node*>& route, Point start, Point destination, const Grid& grid);
}