/*
 * DroneCommandUtils collects validation and geographic calculations shared by
 * the MAVSDK controller. Keeping these small rules in one place makes the main
 * controller easier to read and gives connection ports and distance checks one
 * consistent definition.
 */

#pragma once

#include "../DroneAPI.h"

#include <string>

// All methods are stateless because they only validate values or perform small
// deterministic conversions.
class DroneCommandUtils {
public:
    static bool validDroneID(int droneID);
    static int portForDrone(int droneID);
    static std::string connectionUrlForDrone(int droneID);

    static bool validTakeoffAltitude(double altitudeMeters);
    static bool validLocationTarget(DroneAPI::LocationTarget target);

    static double distanceMeters(
        double latitudeA,
        double longitudeA,
        double latitudeB,
        double longitudeB
    );

private:
    static double toRadians(double degrees);
};
