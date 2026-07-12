/*
 * These helpers cover the small pieces of command validation and coordinate
 * math that do not need access to a live MAVSDK system. In particular, the
 * distance function uses the haversine formula so target checks work with GPS
 * coordinates rather than a flat local grid.
 */

#include "DroneCommandUtils.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numbers>
#include <iomanip>

namespace {
    double metersToFeetForDisplay(double meters) {
        return meters / 0.3048;
    }
}

bool DroneCommandUtils::validDroneID(int droneID) {
    return droneID >= 1;
}

int DroneCommandUtils::portForDrone(int droneID) {
    return 14540 + (droneID - 1);
}

std::string DroneCommandUtils::connectionUrlForDrone(int droneID) {
    return "udpin://0.0.0.0:" + std::to_string(portForDrone(droneID));
}

bool DroneCommandUtils::validTakeoffAltitude(double altitudeMeters) {
    if (!std::isfinite(altitudeMeters)) {
        std::cerr << "Takeoff altitude must be a real number.\n";
        return false;
    }

    if (altitudeMeters <= 0.0) {
        std::cerr << "Takeoff altitude must be greater than 0.\n";
        return false;
    }

    return true;
}

bool DroneCommandUtils::validLocationTarget(DroneAPI::LocationTarget target) {
    if (!std::isfinite(target.latitude)) {
        std::cerr << "Invalid latitude. Latitude must be a real number.\n";
        return false;
    }

    if (!std::isfinite(target.longitude)) {
        std::cerr << "Invalid longitude. Longitude must be a real number.\n";
        return false;
    }

    if (!std::isfinite(target.relativeAltitudeMeters)) {
        std::cerr << "Invalid relative altitude. Altitude must be a real number.\n";
        return false;
    }

    if (target.latitude < -90.0 || target.latitude > 90.0) {
        std::cerr << "Invalid latitude. Latitude must be between -90 and 90.\n";
        return false;
    }

    if (target.longitude < -180.0 || target.longitude > 180.0) {
        std::cerr << "Invalid longitude. Longitude must be between -180 and 180.\n";
        return false;
    }

    if (target.relativeAltitudeMeters < 1.0) {
    	std::cerr << std::fixed << std::setprecision(1)
    			  << "Relative altitude should be at least "
    			  << metersToFeetForDisplay(1.0)
    			  << " feet for flight commands.\n";
    	return false;
    }

    return true;
}

double DroneCommandUtils::toRadians(double degrees) {
    return degrees * (std::numbers::pi / 180.0);
}

// Haversine distance is accurate enough for the short and medium ranges this
// project checks, and it avoids assuming the Earth is flat around the home point.
double DroneCommandUtils::distanceMeters(
    double latitudeA,
    double longitudeA,
    double latitudeB,
    double longitudeB
) {
    constexpr double earthRadiusMeters = 6371000.0;

    double lat1 = toRadians(latitudeA);
    double lat2 = toRadians(latitudeB);

    double deltaLat = toRadians(latitudeB - latitudeA);
    double deltaLon = toRadians(longitudeB - longitudeA);

    double sinDeltaLat = std::sin(deltaLat / 2.0);
    double sinDeltaLon = std::sin(deltaLon / 2.0);

    double h =
        sinDeltaLat * sinDeltaLat +
        std::cos(lat1) * std::cos(lat2) *
        sinDeltaLon * sinDeltaLon;

    h = std::clamp(h, 0.0, 1.0);

    double c =
        2.0 * std::atan2(std::sqrt(h), std::sqrt(1.0 - h));

    return earthRadiusMeters * c;
}
