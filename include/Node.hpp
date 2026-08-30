#pragma once

namespace SailingEngine {

    enum class TerrainType {
        WATER,
        LAND,
        RESTRICTED // Geofenced boundaries (e.g., military zones, 300m beach buffers)
    };

    struct Node {
        int x;
        int y;
        TerrainType type;
        double depth; // Water depth in meters
        
        // Pathfinding state variables
        double gCost; // Known cost from start to this node
        double hCost; // Heuristic cost to target
        Node* parent; // Pointer for path reconstruction

        Node(int x_coord, int y_coord, TerrainType t = TerrainType::WATER, double nodeDepth = 10.0)
            : x(x_coord), y(y_coord), type(t), depth(nodeDepth), gCost(0.0), hCost(0.0), parent(nullptr) {}

        // F-Cost: Total path cost evaluation
        double fCost() const {
            return gCost + hCost;
        }

        // Determines tile passability based on terrain rules and vessel constraints
        bool isNavigable(double vesselDraft) const {
            // Evaluate terrain restrictions (impassable zones)
            if (type == TerrainType::LAND || type == TerrainType::RESTRICTED) {
                return false;
            }
            
            // Depth clearance check: requires a 0.5m safety margin beneath the keel
            const double clearanceMargin = 0.5; 
            return depth >= (vesselDraft + clearanceMargin);
        }
    };

} // namespace SailingEngine