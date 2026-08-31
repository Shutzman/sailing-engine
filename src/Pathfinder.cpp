#include "../include/Pathfinder.hpp"
#include <cmath>
#include <algorithm>
#include <unordered_set>
#include <iostream>

// Ensure M_PI is defined (some MSVC configurations omit it by default)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace SailingEngine {

    Pathfinder::Pathfinder(Grid& gridRef) : grid(gridRef) {}

    double Pathfinder::calculateHeuristic(Node* a, Node* b) const {
        double dx = a->pos.x - b->pos.x;
        double dy = a->pos.y - b->pos.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    double Pathfinder::calculateHeading(int dx, int dy) const {
        // Cartesian grid to Compass heading:
        // dx is East (+), dy is South (+)
        // atan2(dx, -dy) maps (0, -1) to 0 rad (North), (1, 0) to pi/2 rad (East)
        double heading = std::atan2(dx, -dy) * (180.0 / M_PI);
        if (heading < 0.0) {
            heading += 360.0;
        }
        return heading;
    }

    double Pathfinder::getWindCostMultiplier(double heading, const Wind& wind, PropulsionType propulsion) const {
        // Calculate True Wind Angle (TWA) - difference between heading and wind direction
        double diff = std::fmod(std::abs(heading - wind.directionDegrees), 360.0);
        double twa = diff > 180.0 ? 360.0 - diff : diff; // Normalize to [0, 180] degrees

        // The No-Go Zone (Direct Headwind)
        if (twa < 45.0) {
            if (propulsion == PropulsionType::SAIL_ONLY) {
                return -1.0; // Impassable
            } else {
                return 2.0; // High resistance penalty for engines pushing into wind
            }
        }
        // Close-Hauled (Upwind)
        else if (twa < 60.0) {
            return 1.5; // Slower progress due to leeway and heeling
        }
        // Reaching (Wind across the beam) - Fastest point of sail
        else if (twa < 120.0) {
            return 0.8; // Efficiency discount
        }
        // Running Downwind
        else {
            return 1.0; // Good speed, but generally slightly slower than a broad reach
        }
    }

    std::vector<Node*> Pathfinder::retracePath(Node* startNode, Node* endNode) const {
        std::vector<Node*> path;
        Node* currentNode = endNode;
        
        while (currentNode != startNode) {
            path.push_back(currentNode);
            currentNode = currentNode->parent;
        }
        
        std::reverse(path.begin(), path.end());
        return path;
    }

    std::vector<Node*> Pathfinder::findPath(Point start, Point target, const Vessel& vessel) {
        Node* startNode = grid.getNode(start);
        Node* targetNode = grid.getNode(target);

        if (!startNode || !targetNode) {
            std::cerr << "Error: Start or Target coordinates are out of bounds.\n";
            return {};
        }

        if (!startNode->isNavigable(vessel.getDraft()) || !targetNode->isNavigable(vessel.getDraft())) {
            std::cerr << "Error: Start or Target tile is impassable for this vessel's draft.\n";
            return {};
        }

        std::vector<Node*> openSet;
        std::unordered_set<Node*> closedSet;

        startNode->gCost = 0.0;
        startNode->hCost = calculateHeuristic(startNode, targetNode);

        openSet.push_back(startNode);

        while (!openSet.empty()) {
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

            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    if (dx == 0 && dy == 0) continue;

                    Point checkPos{currentNode->pos.x + dx, currentNode->pos.y + dy};
                    Node* neighbor = grid.getNode(checkPos);

                    if (!neighbor || closedSet.count(neighbor) || !neighbor->isNavigable(vessel.getDraft())) {
                        continue;
                    }

                    // Environmental cost application
                    double heading = calculateHeading(dx, dy);
                    Wind localWind = grid.getWindAt(checkPos);
                    double windMultiplier = getWindCostMultiplier(heading, localWind, vessel.getPropulsion());

                    // Negative multiplier indicates physically impossible headings
                    if (windMultiplier < 0) {
                        continue;
                    }

                    double baseMoveCost = (dx != 0 && dy != 0) ? 1.414 : 1.0;
                    double newCostToNeighbor = currentNode->gCost + (baseMoveCost * windMultiplier);

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

        // std::cout << "No valid path could be found.\n"; // If isEmpty() then no route was found 
        return {};
    }
} // namespace SailingEngine