#pragma once

namespace SailingEngine {

    struct Wind {
        double directionDegrees; // 0 = North (blowing towards South), 90 = East, 180 = South, 270 = West
        double speedKnots;

        Wind(double direction = 0.0, double speed = 10.0)
            : directionDegrees(direction), speedKnots(speed) {}
    };

} // namespace SailingEngine