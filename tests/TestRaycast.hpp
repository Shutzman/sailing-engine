#pragma once

#include <iostream>
#include <cassert>
#include <cmath>
#include "../include/RaycastTraversal.hpp"

namespace SailingEngine::Tests {

    constexpr double RAY_TOLERANCE = 1e-6;

    inline void testRayDtSumInvariant() {
        // Test arbitrary angles: the sum of dt fractions across cells MUST equal 1.0
        Point p0{2, 3};
        Point p1{17, 19};

        double dtSum = 0.0;
        bool completed = traverseGridRay(p0, p1, [&](const RayStep& step) {
            dtSum += step.dt;
            return true;
        });

        assert(completed);
        assert(std::abs(dtSum - 1.0) < RAY_TOLERANCE && "Sum of dt along ray must equal 1.0!");
        std::cout << "  [PASS] Raycast dt Sum Invariant\n";
    }

    inline void testRayEarlyAbort() {
        Point p0{0, 0};
        Point p1{10, 0};

        int stepsCount = 0;
        bool completed = traverseGridRay(p0, p1, [&](const RayStep& step) {
            stepsCount++;
            return stepsCount < 4; // Abort on step 4
        });

        assert(!completed && "Traverser must report early abort!");
        assert(stepsCount == 4);
        std::cout << "  [PASS] Raycast Early Abort\n";
    }

    inline void runAllRaycastTests() {
        std::cout << "[RUNNING] RaycastTraversal Test Suite...\n";
        testRayDtSumInvariant();
        testRayEarlyAbort();
        std::cout << "[PASSED] RaycastTraversal Suite Completed.\n\n";
    }

} // namespace SailingEngine::Tests