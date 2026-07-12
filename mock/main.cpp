/*
 * This is a compact smoke test for the mock controller. It exercises the basic
 * connection, arming, takeoff, movement, telemetry, and landing path without
 * running the interactive console application.
 */

// mock
#include "MockDroneController.h"

#include <iostream>
#include <iomanip>

int main() {
    MockDroneController drone;

    if (!drone.connectToDrone(1)) {
        return 1;
    }

    if (!drone.arm()) {
        return 1;
    }

    if (!drone.takeoff(10.0)) {
        return 1;
    }

    // Example target near the Space Needle.
    if (!drone.goToLocation({47.620422, -122.349358, 30.0})) {
        return 1;
    }

    std::cout << "Battery life: " << drone.batteryLife() << "%\n";

    DroneAPI::CurrentLocation location = drone.currentLocation();
	
	std::cout << std::fixed << std::setprecision(6);
	
    std::cout << "Current location: "
              << location.latitude << ", "
              << location.longitude
              << " relative altitude: "
              << location.relativeAltitudeMeters
              << " meters"
              << " absolute altitude: "
              << location.absoluteAltitudeMeters
              << " meters AMSL\n";

    if (!drone.land()) {
        return 1;
    }

    return 0;
}
