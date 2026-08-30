#include "../include/Grid.hpp"
#include <stdexcept>
#include <algorithm>

namespace SailingEngine {

    Grid::Grid(int w, int h) : width(w), height(h) {
        initializeGrid();
    }

    void Grid::initializeGrid() {
        // Allocate columns along the X axis for nodes
        gridData.resize(width);
        
        // Initialize the wind layer with a default 10kt North wind across the entire grid
        windLayer.resize(width, std::vector<Wind>(height, Wind(0.0, 10.0))); 

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

    void Grid::setWindAt(int x, int y, const Wind& wind) {
        if (x >= 0 && x < width && y >= 0 && y < height) {
            windLayer[x][y] = wind;
        } else {
            throw std::out_of_range("Coordinates out of grid bounds.");
        }
    }

    void Grid::setUniformWind(const Wind& wind) {
        for (int x = 0; x < width; ++x) {
            for (int y = 0; y < height; ++y) {
                windLayer[x][y] = wind;
            }
        }
    }

    void Grid::setWindArea(int x1, int x2, int y1, int y2, const Wind& wind) {
        // Calculate safe boundaries, clamping to grid edges and handling reversed inputs
        int minX = std::max(0, std::min(x1, x2));
        int maxX = std::min(width - 1, std::max(x1, x2));
        int minY = std::max(0, std::min(y1, y2));
        int maxY = std::min(height - 1, std::max(y1, y2));

        for (int x = minX; x <= maxX; ++x) {
            for (int y = minY; y <= maxY; ++y) {
                windLayer[x][y] = wind;
            }
        }
    }

    Node* Grid::getNode(int x, int y) const {
        if (x >= 0 && x < width && y >= 0 && y < height) {
            // Return raw non-owning pointer for inspection without transferring ownership
            return gridData[x][y].get();
        }
        return nullptr;
    }

    Wind Grid::getWindAt(int x, int y) const {
        if (x >= 0 && x < width && y >= 0 && y < height) {
            return windLayer[x][y];
        }
        return Wind(); // Fallback to default wind if out of bounds
    }

} // namespace SailingEngine