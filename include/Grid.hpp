#pragma once
#include <vector>
#include <memory>
#include "Node.hpp"

namespace SailingEngine {

    class Grid {
    private:
        int width;
        int height;
        
        // 2D grid matrix. Using std::unique_ptr to ensure strict memory ownership 
        // by the Grid and prevent memory leaks upon destruction.
        std::vector<std::vector<std::unique_ptr<Node>>> gridData;

        // Internal initialization routine called during construction
        void initializeGrid();

    public:
        // Constructor
        Grid(int w, int h);
        
        // Deleted copy constructor and assignment operator.
        // Prevents accidental, expensive deep copies of large grid data (e.g., passing by value).
        Grid(const Grid&) = delete;
        Grid& operator=(const Grid&) = delete;

        // Modifiers
        void setTerrainType(int x, int y, TerrainType type);
        void setDepth(int x, int y, double newDepth);
        
        // Getters
        Node* getNode(int x, int y) const;
        int getWidth() const { return width; }
        int getHeight() const { return height; }
    };

} // namespace SailingEngine