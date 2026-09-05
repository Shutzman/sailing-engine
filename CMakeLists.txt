cmake_minimum_required(VERSION 3.12)
project(sailing_engine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 1. Engine Core Library (Compiles sources once)
set(SOURCES 
    src/Grid.cpp
    src/Pathfinder.cpp
    src/Visualizer.cpp
    src/Exporter.cpp
    src/NauticalCostModel.cpp
)

add_library(sailing_engine_core STATIC ${SOURCES})
target_include_directories(sailing_engine_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

if(MSVC)
    target_compile_options(sailing_engine_core PRIVATE /W4)
else()
    target_compile_options(sailing_engine_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

# 2. Main Production Executable
add_executable(SailingEngine src/main.cpp)
target_link_libraries(SailingEngine PRIVATE sailing_engine_core)

# 3. Master Test Executable
add_executable(SailingEngineTests tests/mainTest.cpp)
target_link_libraries(SailingEngineTests PRIVATE sailing_engine_core)

enable_testing()
add_test(NAME EngineTestSuite COMMAND SailingEngineTests)