#include "../include/Grid.hpp"
#include <stdexcept>

namespace SailingEngine {

    Grid::Grid(int w, int h) : width(w), height(h) {
        initializeGrid();
    }

    void Grid::initializeGrid() {
        // Allocate columns along the X axis
        gridData.resize(width);
        
        for (int x = 0; x < width; ++x) {
            // Pre-allocate row capacity along the Y axis to avoid dynamic reallocations
            gridData[x].reserve(height);
            for (int y = 0; y < height; ++y) {
                // Construct Nodes with unique ownership managed by the grid container
                gridData[x].push_back(std::make_unique<Node>(x, y));
            }
        }
    }

    void Grid::setTerrainType(int x, int y, TerrainType type) {
        if (x >= 0 && x < width && y >= 0 && y < height) {
            gridData[x][y]->type = type;
        } else {
            throw std::out_of_range("Coordinates out of grid bounds.");
        }
    }

    void Grid::setDepth(int x, int y, double newDepth) {
        if (x >= 0 && x < width && y >= 0 && y < height) {
            gridData[x][y]->depth = newDepth;
        } else {
            throw std::out_of_range("Coordinates out of grid bounds.");
        }
    }

    Node* Grid::getNode(int x, int y) const {
        if (x >= 0 && x < width && y >= 0 && y < height) {
            // Return raw non-owning pointer for inspection without transferring ownership
            return gridData[x][y].get();
        }
        return nullptr;
    }

} // namespace SailingEngine