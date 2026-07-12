/*
 * MavsdkTelemetryUtils translates MAVSDK telemetry into the project’s own data
 * types. It also gives the controller one consistent way to recognize invalid
 * positions and represent unavailable location data.
 */

#pragma once

#include "../DroneAPI.h"

#include <mavsdk/plugins/telemetry/telemetry.h>

// Centralizing these conversions keeps NaN handling and field mapping consistent.
class MavsdkTelemetryUtils {
public:
    static DroneAPI::CurrentLocation positionToCurrentLocation(
        mavsdk::Telemetry::Position position
    );

    static DroneAPI::CurrentLocation invalidCurrentLocation();

    static bool validCurrentLocation(DroneAPI::CurrentLocation location);

    static bool validTelemetryPosition(
        mavsdk::Telemetry::Position position
    );
};
