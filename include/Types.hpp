#pragma once

namespace SailingEngine {

    struct Point {
        int x{0};
        int y{0};

        constexpr Point() = default;
        constexpr Point(int xVal, int yVal) : x(xVal), y(yVal) {}

        bool operator==(const Point& other) const {
            return x == other.x && y == other.y;
        }

        bool operator!=(const Point& other) const {
            return !(*this == other);
        }
    };

} // namespace SailingEngine