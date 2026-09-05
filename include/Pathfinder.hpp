#pragma once

#include <vector>
#include <optional>
#include "Grid.hpp"
#include "Node.hpp"
#include "Vessel.hpp"
#include "NauticalCostModel.hpp"
#include "RaycastTraversal.hpp"

namespace SailingEngine {

    struct CourseResult {
        double totalCost = 0.0;
        bool engineUsed = false;
    };

    class Pathfinder {
    public:
        explicit Pathfinder(Grid& gridRef);

        std::vector<Node*> findPath(Point start, Point target, const Vessel& vessel);

    private:
        Grid& grid;
        NauticalCostModel costModel;

        // Navigation and metric helpers
        double calculateHeuristic(Node* a, Node* b) const;
        double calculateHeading(int dx, int dy) const;
        double calculateDistance(Point a, Point b) const;

        // Environmental evaluation along continuous segments
        std::optional<StepCostResult> evaluateCell(Point cell, double heading, const Vessel& vessel) const;
        std::optional<CourseResult> checkCourse(Node* fromNode, Node* toNode, const Vessel& vessel) const;

        // Dynamic steering and maneuver costs
        double calculateManeuverCost(Node* evaluationNode, double newHeading, const Wind& localWind) const;

        // Search state mutations and path tracking
        void updateNodeState(Node* node, Node* parent, Node* destination, double newCost, bool engineActive) const;
        std::vector<Node*> retracePath(Node* startNode, Node* endNode) const;
        std::vector<Node*>::iterator getBestNodeIt(std::vector<Node*>& openSet) const;
    };

} // namespace SailingEngine