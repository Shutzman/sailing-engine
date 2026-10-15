# Autonomous Sailing Pathfinding Engine

## 🎯 What This Is

A high-performance C++ pathfinding engine tailored for maritime navigation. This engine calculates optimal, physically realistic routes across a 2D grid by evaluating heterogeneous environmental vector fields (wind direction, land masses, water depth) against specific vessel capabilities.

Instead of relying on standard discrete A* grid steps, the system utilizes continuous computational geometry (Theta* with Amanatides-Woo raycasting) to find the most direct, sailable routes.

## 🛠️ Core Capabilities

- **Nautical Physics Heuristic:** Evaluates True Wind Angle (TWA) to prevent illegal headwind routing, forcing realistic upwind tacking maneuvers.
- **Continuous Raycasting:** Replaces jagged grid-stepping with smooth line-of-sight shortcuts across dynamic environmental zones.
- **Environmental Vector Fields:** Simulates complex macro-environments including localized wind shadows, thermal shifts, and varying channel widths.
- **Multi-Language Pipeline:** The C++ backend calculates the route and serializes the environment into JSON, which is then rendered by a custom Python visualizer into nautical charts with dynamic wind quiver plots.

## 📍 Phase 1 Roadmap

The current development phase focuses on finalizing the core routing logic and physics integration:

- [x] Any-Angle Theta\* Pathfinding implementation
- [x] Voxel raycasting for environmental evaluation
- [x] Python-based visualization and JSON serialization pipeline
- [ ] **Hybrid Propulsion State Machine:** Allowing vessels to dynamically drop sails and engage an engine when trapped in un-sailable headwind channels.
- [ ] Integration of simulated ocean currents into the cost heuristic.

---

## 🚀 Quick Start Guide

Follow these steps to build the C++ engine, generate a route, and visualize the nautical chart.

### Prerequisites

- **C++17 Compiler** (MSVC, GCC, or Clang)
- **CMake** (3.10 or higher)
- **Python 3.x** with `matplotlib` installed (`pip install matplotlib`)

### 1. Build the C++ Engine

Clone the repository and build the project using CMake:

```bash
git clone [https://github.com/Shutzman/sailing-engine.git](https://github.com/Shutzman/sailing-engine.git)
cd sailing-engine
mkdir build
cd build
cmake ..
cmake --build .
```
