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

    double Pathfinder::getWindCostMultiplier(double heading, const Wind& wind, PropulsionType propulsion, bool& engineActive) const {
            constexpr double TWA_NO_GO = 45.0;
            constexpr double TWA_CLOSE_HAULED = 60.0;
            constexpr double TWA_BEAM_REACH = 120.0;

            constexpr double SPEED_BECALMED = 3.0;
            constexpr double SPEED_OPTIMAL = 20.0;
            constexpr double SPEED_HEAVY_WEATHER = 25.0;
            constexpr double SPEED_STORM = 45.0;
            constexpr double SPEED_HURRICANE = 50.0;

            constexpr double COST_IMPASSABLE = -1.0;
            
            // Calculate True Wind Angle (TWA)
            double diff = std::fmod(std::abs(heading - wind.directionDegrees), 360.0);
            double twa = diff > 180.0 ? 360.0 - diff : diff; // Normalize True Wind Angle [0, 180]
            double speed = wind.speedKnots;       
            double costMultiplier = 1.0;

            engineActive = (propulsion == PropulsionType::ENGINE_ONLY); // Always true for a boat without sails

            // --- 3. EXTREME WEATHER OVERRIDES ---
            if (speed < SPEED_BECALMED) {
                if (propulsion == PropulsionType::SAIL_ONLY) {
                    return COST_IMPASSABLE;
                }
                engineActive = true; // HYBRID is forced to turn on the engine
                return 1.0;
            }

            // --- 4. POINT OF SAIL PHASES ---
            if (twa < TWA_NO_GO) {
                if (propulsion == PropulsionType::SAIL_ONLY) {
                    costMultiplier = COST_IMPASSABLE;   
                } else {
                    // Engine fights wind head-on
                    engineActive = true;
                    costMultiplier = 1.0 + (speed / 10.0); 
                }
            }
            else if (twa < TWA_CLOSE_HAULED) {
                if (propulsion == PropulsionType::SAIL_ONLY) {
                    costMultiplier = 1.5; // Heavy heeling penalty
                } else {
                    costMultiplier = 1.0 + (speed / 15.0); // Engine pushing angled wind
                }
                
            }
            else if (twa < TWA_BEAM_REACH) {
                if (propulsion == PropulsionType::SAIL_ONLY || propulsion == PropulsionType::HYBRID) {
                    // Optimal sailing phase
                    costMultiplier = (speed >= SPEED_OPTIMAL && speed < SPEED_STORM) ? 0.6 : 0.8;
                } else {
                    // Heavy side-seas cause rolling for motorboats
                    costMultiplier = (speed >= SPEED_HEAVY_WEATHER) ? 1.2 : 1.0;
                }
            }
            else {
                // Running Downwind
                if (propulsion == PropulsionType::SAIL_ONLY) {
                    costMultiplier = (speed >= SPEED_OPTIMAL) ? 0.8 : 1.0; 
                }
            }

            return costMultiplier;
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
                    bool engineActive = false;
                    double windMultiplier = getWindCostMultiplier(heading, localWind, vessel.getPropulsion(), engineActive);

                    // Negative multiplier indicates physically impossible headings
                    if (windMultiplier < 0) {
                        continue;
                    }

                    double turnPenalty = 0.0;
                    
                    // We can only calculate a turn if we have moved at least one tile from the start
                    if (currentNode->parent != nullptr) {
                        // Calculate the dx/dy of our PREVIOUS step
                        int oldDx = currentNode->pos.x - currentNode->parent->pos.x;
                        int oldDy = currentNode->pos.y - currentNode->parent->pos.y;
                        
                        // If our old trajectory doesn't match our new intended trajectory, we turned!
                        if (oldDx != dx || oldDy != dy) {
                            // Flat penalty added to the route cost. 
                            // 2.0 roughly equals the cost of sailing two extra straight tiles.
                            turnPenalty = 2.0; 
                        }
                    }

                    // --- FINAL COST CALCULATION ---
                    double baseMoveCost = (dx != 0 && dy != 0) ? 1.414 : 1.0;
                    
                    // Add the turn penalty to the total cost to step into this neighbor
                    double newCostToNeighbor = currentNode->gCost + (baseMoveCost * windMultiplier) + turnPenalty;

                    bool inOpenSet = std::find(openSet.begin(), openSet.end(), neighbor) != openSet.end();

                    if (newCostToNeighbor < neighbor->gCost || !inOpenSet) {
                        neighbor->gCost = newCostToNeighbor;
                        neighbor->hCost = calculateHeuristic(neighbor, targetNode);
                        neighbor->parent = currentNode;
                        neighbor->usedEngine = engineActive;

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