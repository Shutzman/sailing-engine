#pragma once
#include <vector>
#include "Grid.hpp"
#include "Vessel.hpp"
#include "Node.hpp"

namespace SailingEngine {

    class Pathfinder {
    private:
        // Reference to the environment grid to avoid expensive data copies
        Grid& grid; 

        // Calculates the heuristic cost (H-Cost) between two nodes
        double calculateHeuristic(Node* startNode, Node* targetNode) const;

        // Reconstructs the final path by tracing parent pointers backwards
        std::vector<Node*> retracePath(Node* startNode, Node* endNode) const;

    public:
        Pathfinder(Grid& gridRef);

        // Core A* pathfinding algorithm implementation.
        // Returns the optimal route as a vector of Nodes, or an empty vector if no valid path exists.
        std::vector<Node*> findPath(int startX, int startY, int targetX, int targetY, const Vessel& vessel);
    };

} // namespace SailingEngine