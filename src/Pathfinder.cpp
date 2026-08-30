#include "../include/Pathfinder.hpp"
#include <cmath>
#include <algorithm>
#include <unordered_set>
#include <iostream>

namespace SailingEngine {

    Pathfinder::Pathfinder(Grid& gridRef) : grid(gridRef) {}

    double Pathfinder::calculateHeuristic(Node* a, Node* b) const {
        // Standard Euclidean distance metric
        double dx = a->x - b->x;
        double dy = a->y - b->y;
        return std::sqrt(dx * dx + dy * dy);
    }

    std::vector<Node*> Pathfinder::retracePath(Node* startNode, Node* endNode) const {
        std::vector<Node*> path;
        Node* currentNode = endNode;
        
        while (currentNode != startNode) {
            path.push_back(currentNode);
            currentNode = currentNode->parent;
        }
        
        // Invert path to establish forward trajectory from start to destination
        std::reverse(path.begin(), path.end());
        return path;
    }

    std::vector<Node*> Pathfinder::findPath(int startX, int startY, int targetX, int targetY, const Vessel& vessel) {
        Node* startNode = grid.getNode(startX, startY);
        Node* targetNode = grid.getNode(targetX, targetY);

        // Boundary constraint verification
        if (!startNode || !targetNode) {
            std::cerr << "Error: Start or Target coordinates are out of bounds.\n";
            return {};
        }

        // Vessel draft navigability validation at endpoints
        if (!startNode->isNavigable(vessel.getDraft()) || !targetNode->isNavigable(vessel.getDraft())) {
            std::cerr << "Error: Start or Target tile is impassable for this vessel's draft.\n";
            return {};
        }

        std::vector<Node*> openSet;
        // Hash set for O(1) closed-set lookups
        std::unordered_set<Node*> closedSet;

        openSet.push_back(startNode);

        while (!openSet.empty()) {
            // Retrieve node with lowest total estimated cost (fCost), breaking ties with hCost
            auto currentNodeIt = openSet.begin();
            for (auto it = openSet.begin(); it != openSet.end(); ++it) {
                if ((*it)->fCost() < (*currentNodeIt)->fCost() || 
                   ((*it)->fCost() == (*currentNodeIt)->fCost() && (*it)->hCost < (*currentNodeIt)->hCost)) {
                    currentNodeIt = it;
                }
            }

            Node* currentNode = *currentNodeIt;

            openSet.erase(currentNodeIt);
            closedSet.insert(currentNode);

            if (currentNode == targetNode) {
                return retracePath(startNode, targetNode);
            }

            // Evaluate 8-directional adjacent nodes
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    if (dx == 0 && dy == 0) continue;

                    int checkX = currentNode->x + dx;
                    int checkY = currentNode->y + dy;
                    Node* neighbor = grid.getNode(checkX, checkY);

                    // Skip unnavigable, visited, or out-of-bound nodes
                    if (!neighbor || closedSet.count(neighbor) || !neighbor->isNavigable(vessel.getDraft())) {
                        continue;
                    }

                    // Geometric traversal cost: 1.0 cardinal, sqrt(2) diagonal
                    double moveCost = (dx != 0 && dy != 0) ? 1.414 : 1.0;
                    double newCostToNeighbor = currentNode->gCost + moveCost;

                    bool inOpenSet = std::find(openSet.begin(), openSet.end(), neighbor) != openSet.end();

                    if (newCostToNeighbor < neighbor->gCost || !inOpenSet) {
                        neighbor->gCost = newCostToNeighbor;
                        neighbor->hCost = calculateHeuristic(neighbor, targetNode);
                        neighbor->parent = currentNode;

                        if (!inOpenSet) {
                            openSet.push_back(neighbor);
                        }
                    }
                }
            }
        }

        std::cout << "No valid path could be found.\n";
        return {};
    }
} // namespace SailingEngine