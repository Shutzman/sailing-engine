#pragma once

namespace SailingEngine {

    enum class PropulsionType {
        SAIL_ONLY,
        ENGINE_ONLY,
        HYBRID
    };

    class Vessel {
    private:
        double lengthOverall;
        double beam;
        double draft;
        PropulsionType propulsion;

    public:
        Vessel(double loa, double b, double d, PropulsionType p)
            : lengthOverall(loa), beam(b), draft(d), propulsion(p) {}

        // Getters
        double getDraft() const { return draft; }
        PropulsionType getPropulsion() const { return propulsion; }
    };

} // namespace SailingEngine