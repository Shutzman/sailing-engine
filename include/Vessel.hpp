#pragma once

namespace SailingEngine {

    enum class PropulsionType {
        SAIL_ONLY,
        ENGINE,
        HYBRID
    };

    class Vessel {
    private:
        double length;     // Length Overall (LOA) in meters
        double width;      // Beam (maximum width) in meters
        double draft;      // Ship's depth below waterline in meters
        PropulsionType propulsion;

    public:
        Vessel(double vesselLength, double vesselWidth, double vesselDraft, PropulsionType propType)
            : length(vesselLength), width(vesselWidth), draft(vesselDraft), propulsion(propType) {}

        double getLength() const { return length; }
        double getWidth() const { return width; }
        double getDraft() const { return draft; }
        PropulsionType getPropulsionType() const { return propulsion; }
    };

} // namespace SailingEngine