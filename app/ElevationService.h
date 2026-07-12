/*
 * ElevationService is a small adapter around the online terrain-elevation
 * lookup used by the simulator setup and the location display. Keeping this
 * work in one class prevents HTTP and response-parsing details from leaking
 * into the rest of the console application.
 */

#pragma once

// The method returns meters so the rest of the code stays consistent with MAVSDK.
class ElevationService {
public:
    static double elevationMeters(double latitude, double longitude);
};
