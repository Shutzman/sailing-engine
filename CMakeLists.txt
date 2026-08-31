cmake_minimum_required(VERSION 3.10)
project(sailing_engine)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. Define the source files containing the actual engine logic
set(ENGINE_SOURCES 
    src/Grid.cpp 
    src/Pathfinder.cpp
    src/Visualizer.cpp
)

# 2. Build the main production executable
add_executable(SailingEngine src/main.cpp ${ENGINE_SOURCES})

# 3. Build the testing executable
add_executable(SailingEngineTests tests/test_pathfinder.cpp ${ENGINE_SOURCES})