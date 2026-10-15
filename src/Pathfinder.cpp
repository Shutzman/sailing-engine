#include "../include/Pathfinder.hpp"
#include <cmath>
#include <algorithm>
#include <unordered_set>
#include <limits>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace SailingEngine {

    Pathfinder::Pathfinder(Grid& gridRef) : grid(gridRef) {}

    // =========================================================================
    // Navigation and Geometry Helpers
    // =========================================================================

    double Pathfinder::calculateDistance(Point a, Point b) const {
        const double dx = static_cast<double>(a.x - b.x);
        const double dy = static_cast<double>(a.y - b.y);
        return std::sqrt(dx * dx + dy * dy);
    }

    double Pathfinder::calculateHeading(int dx, int dy) const {
        // Cartesian grid to compass bearing: dx = East (+), dy = South (+)
        double heading = std::atan2(dx, -dy) * (180.0 / M_PI);
        if (heading < 0.0) {
            heading += 360.0;
        }
        return heading;
    }

    double Pathfinder::calculateHeuristic(Node* a, Node* b) const {
        // Scaling Euclidean distance by the optimal reachable polar multiplier (0.6)
        // guarantees admissibility (h(n) <= c*(n)) across anisotropic wind fields.
        constexpr double MIN_GLOBAL_COST_MULTIPLIER = 0.6;
        return calculateDistance(a->pos, b->pos) * MIN_GLOBAL_COST_MULTIPLIER;
    }

    // =========================================================================
    // Maneuver and Turning Resistance
    // =========================================================================

    double Pathfinder::calculateManeuverCost(Node* evaluationNode, double newHeading, const Wind& localWind, bool usingEngine) const {
        if (!evaluationNode || !evaluationNode->parent) {
            return 0.0;
        }

        const int prevDx = evaluationNode->pos.x - evaluationNode->parent->pos.x;
        const int prevDy = evaluationNode->pos.y - evaluationNode->parent->pos.y;
        const double prevHeading = calculateHeading(prevDx, prevDy);

        const double turnAngle = NauticalCostModel::calculateAngleDifference(newHeading, prevHeading);
        if (turnAngle < 5.0) {
            return 0.0;
        }

        // If motoring, only apply basic rudder drag
        if (usingEngine) {
            return 1.0;
        }

        // Compute signed relative wind angles in [-180.0, 180.0]
        auto getSignedWindAngle = [](double heading, double windDir) {
            return std::fmod(heading - windDir + 540.0, 360.0) - 180.0;
        };

        const double prevSignedTwa = getSignedWindAngle(prevHeading, localWind.directionDegrees);
        const double newSignedTwa  = getSignedWindAngle(newHeading, localWind.directionDegrees);

        // If signs differ (or one hits 0/180 boundary while changing sides): Port <-> Starboard transition
        if ((prevSignedTwa * newSignedTwa) <= 0.0 && (prevSignedTwa != newSignedTwa)) {
            // Determine whether the boat turned through the wind eye (0 deg = Tack)
            // or through the leeward vector (180 deg = Gybe)
            const double avgAbsTwa = 0.5 * (std::abs(prevSignedTwa) + std::abs(newSignedTwa));
            if (avgAbsTwa <= 110.0) {
                return 3.5; // Tacking penalty: inertial loss and sail handling
            } else {
                return 2.0; // Gybing penalty: boom crossover control
            }
        }

        return 1.0; // Standard rudder redirection drag on the same tack
    }

    // =========================================================================
    // Traversal and Cost Integration
    // =========================================================================

    std::optional<StepCostResult> Pathfinder::evaluateCell(Point cell, double heading, const Vessel& vessel) const {
        Node* cellNode = grid.getNode(cell);
        if (!cellNode || !cellNode->isNavigable(vessel.getDraft())) {
            return std::nullopt;
        }

        const Wind cellWind = grid.getWindAt(cell);
        return costModel.calculateStepCost(heading, cellWind, vessel);
    }

    std::optional<CourseResult> Pathfinder::checkCourse(Node* fromNode, Node* toNode, const Vessel& vessel) const {
        const double totalDistance = calculateDistance(fromNode->pos, toNode->pos);
        const double heading = calculateHeading(toNode->pos.x - fromNode->pos.x, toNode->pos.y - fromNode->pos.y);

        CourseResult result;

        const bool completedTraversal = traverseGridRay(fromNode->pos, toNode->pos, [&](const RayStep& step) {
            auto cellEvaluation = evaluateCell(step.cell, heading, vessel);
            if (!cellEvaluation.has_value()) {
                return false; // Physical or aerodynamic obstacle encountered; abort segment
            }

            // Every fraction of the ray (including the 0.5 exit step from fromNode) is accurately billed
            result.totalCost += (step.dt * totalDistance) * cellEvaluation->costMultiplier;
            if (cellEvaluation->engineActive) {
                result.engineUsed = true;
            }

            return true;
        });

        if (!completedTraversal) {
            return std::nullopt;
        }

        return result;
    }

    // =========================================================================
    // Search Graph State and Priority Tracking
    // =========================================================================

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

    void Pathfinder::updateNodeState(Node* node, Node* parent, Node* destination, double newCost, bool engineActive) const {
        node->gCost = newCost;
        node->hCost = calculateHeuristic(node, destination);
        node->parent = parent;
        node->usedEngine = engineActive;
    }

    std::vector<Node*> Pathfinder::retracePath(Node* startNode, Node* endNode) const {
        std::vector<Node*> path;
        Node* currentNode = endNode;

        while (currentNode != nullptr) {
            path.push_back(currentNode);
            if (currentNode == startNode) break;
            currentNode = currentNode->parent;
        }

        std::reverse(path.begin(), path.end());
        return path;
    }

    // =========================================================================
    // Search Execution Loop (Theta* with Competitive Fallback)
    // =========================================================================

    std::vector<Node*> Pathfinder::findPath(Point start, Point target, const Vessel& vessel) {
        Node* startNode = grid.getNode(start);
        Node* targetNode = grid.getNode(target);

        if (!startNode || !targetNode) {
            std::cerr << "Error: Start or Target coordinates out of bounds.\n";
            return {};
        }

        if (!startNode->isNavigable(vessel.getDraft()) || !targetNode->isNavigable(vessel.getDraft())) {
            std::cerr << "Error: Start or Target tile is impassable for vessel draft.\n";
            return {};
        }

        grid.resetSearchState();

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

                    const Point checkPos{ currentNode->pos.x + dx, currentNode->pos.y + dy };
                    Node* neighbor = grid.getNode(checkPos);

                    if (!neighbor || closedSet.count(neighbor) || !neighbor->isNavigable(vessel.getDraft())) {
                        continue;
                    }

                    // Candidate A: Discrete 8-connected grid step from currentNode
                    double candidateCostA = std::numeric_limits<double>::infinity();
                    bool engineUsedA = false;

                    auto stepCourse = checkCourse(currentNode, neighbor, vessel);
                    if (stepCourse.has_value()) {
                        const double headingA = calculateHeading(dx, dy);
                        const Wind localWind = grid.getWindAt(neighbor->pos);
                        const double maneuverA = calculateManeuverCost(currentNode, headingA, localWind, stepCourse->engineUsed);
                        const double hybridPenaltyA = (stepCourse->engineUsed && vessel.getPropulsion() == PropulsionType::HYBRID) ? 5.0 : 0.0;

                        candidateCostA = currentNode->gCost + stepCourse->totalCost + maneuverA + hybridPenaltyA;
                        engineUsedA = stepCourse->engineUsed;
                    }

                    // Candidate B: Continuous any-angle shortcut from currentNode->parent
                    double candidateCostB = std::numeric_limits<double>::infinity();
                    bool engineUsedB = false;

                    if (currentNode->parent != nullptr) {
                        auto shortcutCourse = checkCourse(currentNode->parent, neighbor, vessel);
                        if (shortcutCourse.has_value()) {
                            const int shortcutDx = neighbor->pos.x - currentNode->parent->pos.x;
                            const int shortcutDy = neighbor->pos.y - currentNode->parent->pos.y;
                            const double headingB = calculateHeading(shortcutDx, shortcutDy);
                            const Wind localWind = grid.getWindAt(neighbor->pos);

                            const double maneuverB = calculateManeuverCost(currentNode->parent, headingB, localWind, shortcutCourse->engineUsed);
                            const double hybridPenaltyB = (shortcutCourse->engineUsed && vessel.getPropulsion() == PropulsionType::HYBRID) ? 5.0 : 0.0;

                            candidateCostB = currentNode->parent->gCost + shortcutCourse->totalCost + maneuverB + hybridPenaltyB;
                            engineUsedB = shortcutCourse->engineUsed;
                        }
                    }

                    // Select the cost-optimal valid parent
                    Node* bestParent = nullptr;
                    double bestCandidateCost = std::numeric_limits<double>::infinity();
                    bool bestEngineUsed = false;

                    if (candidateCostB < candidateCostA) {
                        bestParent = currentNode->parent;
                        bestCandidateCost = candidateCostB;
                        bestEngineUsed = engineUsedB;
                    } else if (std::isfinite(candidateCostA)) {
                        bestParent = currentNode;
                        bestCandidateCost = candidateCostA;
                        bestEngineUsed = engineUsedA;
                    }

                    if (!bestParent) {
                        continue;
                    }

                    const bool inOpenSet = std::find(openSet.begin(), openSet.end(), neighbor) != openSet.end();

                    if (bestCandidateCost < neighbor->gCost || !inOpenSet) {
                        updateNodeState(neighbor, bestParent, targetNode, bestCandidateCost, bestEngineUsed);

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