#pragma once

#include <cmath>
#include <algorithm>
#include <limits>
#include "Types.hpp"

namespace SailingEngine {

    struct RayStep {
        Point cell;
        double dt = 0.0;
    };

    /**
     * @brief Performs Amanatides-Woo fast voxel traversal between two discrete points.
     * Evaluates parametric line segment crossings and calls visitor(RayStep) on each visited cell.
     * Halts early if the visitor callback returns false.
     * 
     * @tparam CellVisitor Callable with signature: bool(const RayStep&)
     * @return true if traversal completed to p1 without interruption; false if aborted early.
     */
    template <typename CellVisitor>
    bool traverseGridRay(Point p0, Point p1, CellVisitor&& visitor) {
        if (p0 == p1) {
            return visitor(RayStep{ p0, 0.0 });
        }

        const double dx = static_cast<double>(p1.x - p0.x);
        const double dy = static_cast<double>(p1.y - p0.y);

        const int stepX = (dx > 0.0) ? 1 : (dx < 0.0 ? -1 : 0);
        const int stepY = (dy > 0.0) ? 1 : (dy < 0.0 ? -1 : 0);

        constexpr double INF = std::numeric_limits<double>::infinity();
        const double tDeltaX = (dx != 0.0) ? std::abs(1.0 / dx) : INF;
        const double tDeltaY = (dy != 0.0) ? std::abs(1.0 / dy) : INF;

        // Grid coordinates represent the center of the cell. 
        // A ray starting at the center reaches the first cell boundary at exactly half a step.
        double tMaxX = (dx != 0.0) ? 0.5 * tDeltaX : INF;
        double tMaxY = (dy != 0.0) ? 0.5 * tDeltaY : INF;

        Point currentCell = p0;
        double tCurrent = 0.0;

        while (tCurrent < 1.0) {
            const double tNext = std::min({ tMaxX, tMaxY, 1.0 });
            const double dt = tNext - tCurrent;

            if (!visitor(RayStep{ currentCell, dt })) {
                return false; // Ray hit an obstacle or no-go zone
            }

            if (tMaxX < tMaxY) {
                currentCell.x += stepX;
                tCurrent = tMaxX;
                tMaxX += tDeltaX;
            } else {
                currentCell.y += stepY;
                tCurrent = tMaxY;
                tMaxY += tDeltaY;
            }
        }

        return true;
    }

} // namespace SailingEngine