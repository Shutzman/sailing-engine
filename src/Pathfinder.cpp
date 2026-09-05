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

    bool Pathfinder::hasLineOfSight(Node* fromNode, Node* toNode, double vesselDraft) const {
        Point p0{fromNode->pos.x, fromNode->pos.y};
        const Point p1{toNode->pos.x, toNode->pos.y};

        int dx = std::abs(p1.x - p0.x);
        int dy = std::abs(p1.y - p0.y);
        
        int sx = (p0.x < p1.x) ? 1 : -1;
        int sy = (p0.y < p1.y) ? 1 : -1;
        
        int err = dx - dy;

        while (true) {
            // Check current position using our Point tracker
            Node* checkNode = grid.getNode(p0);
            
            if (!checkNode || !checkNode->isNavigable(vesselDraft)) {
                return false;
            }

            // Reached destination successfully
            if (p0 == p1) {
                break;
            }

            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                p0.x += sx; // Mutate p0.x correctly
            }
            if (e2 < dx) {
                err += dx;
                p0.y += sy; // Mutate p0.y correctly
            }
        }
        
        return true;
    }

    std::vector<Node*>::iterator Pathfinder::getBestNodeIt(std::vector<Node*>& openSet) const {
        auto bestIt = openSet.begin();
        for (auto it = openSet.begin(); it != openSet.end(); ++it) {
            if ((*it)->fCost() < (*bestIt)->fCost() || 
                ((*it)->fCost() == (*bestIt)->fCost() && (*it)->hCost < (*bestIt)->hCost)) {
                bestIt = it;
            }
        }
        return bestIt;
    }

    Node* Pathfinder::getThetaEvaluationNode(Node* currentNode, Node* neighbor, double draft) const {
        if (currentNode->parent != nullptr && hasLineOfSight(currentNode->parent, neighbor, draft)) {
            return currentNode->parent; // Theta* Bypass
        }
        return currentNode; // Standard A* step
    }

    double Pathfinder::calculateTurnPenalty(Node* evaluationNode, double currentHeading) const {
        if (evaluationNode->parent == nullptr) {
            return 0.0;
        }

        int oldDx = evaluationNode->pos.x - evaluationNode->parent->pos.x;
        int oldDy = evaluationNode->pos.y - evaluationNode->parent->pos.y;
        double oldHeading = calculateHeading(oldDx, oldDy);
        
        double angleDiff = std::abs(currentHeading - oldHeading);
        if (angleDiff > 180.0) angleDiff = 360.0 - angleDiff;
        
        return (angleDiff > 5.0) ? 2.0 : 0.0;
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

    void Pathfinder::updateNodeState(Node* node, Node* parent, Node* destination, double newCost, bool engineActive) const {
            node->gCost = newCost;
            node->hCost = calculateHeuristic(node, destination);
            node->parent = parent;
            node->usedEngine = engineActive;
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
            auto currentNodeIt = getBestNodeIt(openSet);
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

                    // 1. Theta* Line-of-Sight Bypass
                    Node* evaluationNode = getThetaEvaluationNode(currentNode, neighbor, vessel.getDraft());

                    // 2. Continuous Vector Math from the actual evaluation node (parent or current)
                    int moveDx = neighbor->pos.x - evaluationNode->pos.x;
                    int moveDy = neighbor->pos.y - evaluationNode->pos.y;
                    
                    double heading = calculateHeading(moveDx, moveDy);
                    
                    // Sample wind at the midpoint or target of the vector for realistic environment impact
                    Wind localWind = grid.getWindAt(neighbor->pos);
                    
                    bool engineActive = false;
                    double windMultiplier = getWindCostMultiplier(heading, localWind, vessel.getPropulsion(), engineActive);

                    if (windMultiplier < 0) {
                        continue; // Physically impossible heading (e.g., direct headwind on sails)
                    }

                    // 3. Calculate Penalties
                    double turnPenalty = calculateTurnPenalty(evaluationNode, heading);
                    double enginePenalty = (engineActive && vessel.getPropulsion() == PropulsionType::HYBRID) ? 5.0 : 0.0;

                    // 4. Final Any-Angle Cost (True Euclidean Distance vector)
                    double baseMoveCost = std::sqrt(moveDx * moveDx + moveDy * moveDy);
                    double newCostToNeighbor = evaluationNode->gCost + (baseMoveCost * windMultiplier) + turnPenalty + enginePenalty;

                    bool inOpenSet = std::find(openSet.begin(), openSet.end(), neighbor) != openSet.end();

                    // 5. Update Neighbor
                    if (newCostToNeighbor < neighbor->gCost || !inOpenSet) {
                        updateNodeState(neighbor, evaluationNode, targetNode, newCostToNeighbor, engineActive);

                        if (!inOpenSet) {
                            openSet.push_back(neighbor);
                        }
                    }
                }
            }
        }

        return {};
    }
} // namespace SailingEngine