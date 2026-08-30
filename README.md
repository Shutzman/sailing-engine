# Autonomous Sailing Pathfinding Engine

## 🎯 Project Overview

A high-performance C++ pathfinding engine tailored for maritime navigation. It calculates optimal routes across a 2D grid by evaluating environmental factors like wind direction, water depth, and vessel capabilities.

## 🟢 In-Scope (Phase 1: Core Engine)

- **2D Grid & ASCII Visualization:** A coordinate map representing terrain, depth, and dynamic obstacles, visualized directly in the terminal.
- **A\* Pathfinding Implementation:** Optimized shortest-path algorithm.
- **Vessel Profiling:** Accounts for ship draft (שוקע) against water depth, and propulsion type (Sail vs. Engine).
- **Wind-Aware Heuristics:** Sail-powered vessels will be penalized or restricted from sailing directly into the wind, requiring realistic tacking routes.
- **Extensible Architecture:** Built using Interfaces (`IEnvironmentalProvider`, `IObstacleProvider`) to allow easy future integration of live Weather APIs and crowd-sourced hazard warnings.
- **Tactical Logging:** Detailed tracking of algorithmic decision-making and performance metrics.

## 🔴 Out-of-Scope (Phase 1)

- **Graphical User Interface (GUI):** No heavy rendering engines (OpenGL, Qt).
- **Live API Connections:** Weather and social hazards are statically mocked for now; live web requests are deferred to Phase 2.
- **Advanced Physics:** Wave drag, momentum, and turning radius inertia are excluded.
