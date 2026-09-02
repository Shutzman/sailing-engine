#pragma once
#include <limits>
#include "Types.hpp"

namespace SailingEngine {

    enum class TerrainType {
        WATER,
        SHALLOWS,
        LAND,
        RESTRICTED
    };

    struct Node {
        Point pos;
        TerrainType type;
        double depth;

        double gCost;
        double hCost;
        Node* parent;
        bool usedEngine;

        Node(Point position, TerrainType t = TerrainType::WATER, double nodeDepth = 10.0)
            : pos(position), type(t), depth(nodeDepth), 
              gCost(0.0), hCost(0.0), parent(nullptr), usedEngine(false) {}

        // Convenience constructor for raw x, y
        Node(int xVal, int yVal, TerrainType t = TerrainType::WATER, double d = 10.0)
            : Node(Point{xVal, yVal}, t, d) {}

        double fCost() const {
            return gCost + hCost;
        }

        bool isNavigable(double vesselDraft) const {
            if (type == TerrainType::LAND || type == TerrainType::RESTRICTED) {
                return false;
            }
            return depth >= vesselDraft;
        }
    };

} // namespace SailingEngine