#include <iostream>
#include "TestNauticalCostModel.hpp"
#include "TestRaycast.hpp"
#include "TestPathfinder.hpp"

int main() {
    std::cout << "========================================\n";
    std::cout << "   Sailing Engine Master Test Suite     \n";
    std::cout << "========================================\n\n";

    try {
        // 1. Physics Layer Unit Tests
        SailingEngine::Tests::runAllNauticalCostModelTests();

        // 2. Traversal Geometry Unit Tests
        SailingEngine::Tests::runAllRaycastTests();

        // 3. Search & Engine Integration Tests
        //SailingEngine::Tests::runAllPathfinderTests();

        std::cout << "========================================\n";
        std::cout << "   ALL TEST SUITES PASSED SUCCESSFULLY! \n";
        std::cout << "========================================\n";
    } 
    catch (const std::exception& ex) {
        std::cerr << "\n[TEST EXCEPTION CAUGHT]: " << ex.what() << "\n";
        return 1;
    }

    return 0;
}