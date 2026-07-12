/*
 * This implementation keeps MAVSDK-specific position handling in one place.
 * Valid readings are copied into DroneAPI structures, while unavailable data is
 * represented with NaN values so callers can detect it without mistaking zero
 * latitude, longitude, or altitude for a real measurement.
 */

#include "MavsdkTelemetryUtils.h"

#include <cmath>
#include <limits>

DroneAPI::CurrentLocation MavsdkTelemetryUtils::positionToCurrentLocation(
    mavsdk::Telemetry::Position position
) {
    return {
        position.latitude_deg,
        position.longitude_deg,
        position.relative_altitude_m,
        position.absolute_altitude_m
    };
}

// NaN is used instead of zero because zero can be a valid coordinate or altitude.
DroneAPI::CurrentLocation MavsdkTelemetryUtils::invalidCurrentLocation() {
    double nan = std::numeric_limits<double>::quiet_NaN();

    return {nan, nan, nan, nan};
}

bool MavsdkTelemetryUtils::validCurrentLocation(
    DroneAPI::CurrentLocation location
) {
    return std::isfinite(location.latitude) &&
           std::isfinite(location.longitude) &&
           std::isfinite(location.relativeAltitudeMeters) &&
           std::isfinite(location.absoluteAltitudeMeters);
}

bool MavsdkTelemetryUtils::validTelemetryPosition(
    mavsdk::Telemetry::Position position
) {
    return std::isfinite(position.latitude_deg) &&
           std::isfinite(position.longitude_deg) &&
           std::isfinite(position.absolute_altitude_m) &&
           std::isfinite(position.relative_altitude_m);
}
