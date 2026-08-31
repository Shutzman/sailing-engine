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
                // Construct Nodes with Point ownership
                gridData[x].push_back(std::make_unique<Node>(Point{x, y}));
            }
        }
    }

    void Grid::setTerrainType(Point p, TerrainType type) {
        if (isInBounds(p)) {
            gridData[p.x][p.y]->type = type;
        } else {
            throw std::out_of_range("Coordinates out of grid bounds.");
        }
    }

    void Grid::setDepth(Point p, double newDepth) {
        if (isInBounds(p)) {
            gridData[p.x][p.y]->depth = newDepth;
        } else {
            throw std::out_of_range("Coordinates out of grid bounds.");
        }
    }

    void Grid::setWindAt(Point p, const Wind& wind) {
        if (isInBounds(p)) {
            windLayer[p.x][p.y] = wind;
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

    void Grid::setWindArea(Point p1, Point p2, const Wind& wind) {
        // Calculate safe boundaries, clamping to grid edges and handling reversed inputs
        int minX = std::max(0, std::min(p1.x, p2.x));
        int maxX = std::min(width - 1, std::max(p1.x, p2.x));
        int minY = std::max(0, std::min(p1.y, p2.y));
        int maxY = std::min(height - 1, std::max(p1.y, p2.y));

        for (int x = minX; x <= maxX; ++x) {
            for (int y = minY; y <= maxY; ++y) {
                windLayer[x][y] = wind;
            }
        }
    }

    Node* Grid::getNode(Point p) const {
        if (isInBounds(p)) {
            // Return raw non-owning pointer for inspection without transferring ownership
            return gridData[p.x][p.y].get();
        }
        return nullptr;
    }

    Wind Grid::getWindAt(Point p) const {
        if (isInBounds(p)) {
            return windLayer[p.x][p.y];
        }
        return Wind(); // Fallback to default wind if out of bounds
    }

} // namespace SailingEngine