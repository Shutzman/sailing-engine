#pragma once
#include <vector>
#include "Grid.hpp"
#include "Node.hpp"
#include "Vessel.hpp"
#include "Environment.hpp"

namespace SailingEngine {

    class Pathfinder {
    private:
        Grid& grid;
        
        double calculateHeuristic(Node* a, Node* b) const;
        std::vector<Node*> retracePath(Node* startNode, Node* endNode) const;
        
        // Navigation vector mathematics
        double calculateHeading(int dx, int dy) const;
        double getWindCostMultiplier(double heading, const Wind& wind, PropulsionType propulsion) const;

    public:
        explicit Pathfinder(Grid& gridRef);
        
        std::vector<Node*> findPath(int startX, int startY, int targetX, int targetY, const Vessel& vessel);
    };

} // namespace SailingEngine