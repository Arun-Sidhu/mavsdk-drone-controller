/*
 * SafetyLimits holds the development guardrails used by the MAVSDK controller.
 * These values help catch accidental commands while testing, but they are not a
 * substitute for PX4 configuration, manufacturer guidance, local regulations,
 * or a real preflight safety process.
 */

#pragma once

// The controller owns an instance of this structure so the limits are easy to
// find and can later be loaded from configuration without changing DroneAPI.
struct SafetyLimits {
    double maxHorizontalDistanceMeters = 3048; // 10000 ft
    double maxRelativeAltitudeMeters = 30.48; // 100 ft
    double minRelativeAltitudeMeters = 3.048; // 10 ft

    double maxVelocityMetersPerSecond = 1.524; // 5 ft/s
    double maxVerticalSpeedMetersPerSecond = 0.6096; // 2 ft/s
    double maxYawSpeedDegreesPerSecond = 30.0;
    double maxVelocityDurationSeconds = 5.0;
};
