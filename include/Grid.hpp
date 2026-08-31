#pragma once
#include <vector>
#include <memory>
#include "Node.hpp"
#include "Environment.hpp"
#include "Types.hpp"

namespace SailingEngine {

    class Grid {
    private:
        int width;
        int height;
        
        // 2D grid matrix. Using std::unique_ptr to ensure strict memory ownership 
        // by the Grid and prevent memory leaks upon destruction.
        std::vector<std::vector<std::unique_ptr<Node>>> gridData;

        // Environmental wind layer parallel to gridData
        std::vector<std::vector<Wind>> windLayer;

        // Internal initialization routine called during construction
        void initializeGrid();

    public:
        Grid(int w, int h);

        // Deleted copy constructor and assignment operator.
        // Prevents accidental, expensive deep copies of large grid data (e.g., passing by value).
        Grid(const Grid&) = delete;
        Grid& operator=(const Grid&) = delete;

        // Setters using Point
        void setTerrainType(Point p, TerrainType type);
        void setDepth(Point p, double newDepth);
        void setWindAt(Point p, const Wind& wind);
        void setWindArea(Point p1, Point p2, const Wind& wind);
        void setUniformWind(const Wind& wind);

        // Backward compatibility overloads for (int, int)
        void setTerrainType(int x, int y, TerrainType type) { setTerrainType(Point{x, y}, type); }
        void setDepth(int x, int y, double newDepth) { setDepth(Point{x, y}, newDepth); }
        void setWindAt(int x, int y, const Wind& wind) { setWindAt(Point{x, y}, wind); }
        void setWindArea(int x1, int y1, int x2, int y2, const Wind& wind) { setWindArea(Point{x1, y1}, Point{x2, y2}, wind); }

        // Getters using Point
        Node* getNode(Point p) const;
        Wind getWindAt(Point p) const;

        // Backward compatibility getters for (int, int)
        Node* getNode(int x, int y) const { return getNode(Point{x, y}); }
        Wind getWindAt(int x, int y) const { return getWindAt(Point{x, y}); }

        int getWidth() const { return width; }
        int getHeight() const { return height; }

        // Boundary check helper
        bool isInBounds(Point p) const {
            return p.x >= 0 && p.x < width && p.y >= 0 && p.y < height;
        }
    };

} // namespace SailingEngine