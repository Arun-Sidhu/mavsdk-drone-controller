/*
 * MockDroneController is a small stateful stand-in for a real vehicle. It
 * implements the same DroneAPI interface as the MAVSDK controller, which makes
 * it useful for checking application flow without launching PX4 or connecting
 * hardware. It is not a physics simulator and moves between states instantly.
 */

#pragma once

#include "../DroneAPI.h"

#include <string>
#include <vector>

// This class follows the production interface closely, but it deliberately avoids
// threads, timing, networking, and external dependencies.
class MockDroneController : public DroneAPI {
public:
    bool connectToDrone(int droneID = 1) override;
    bool connectToDrone(const std::string& connectionUrl) override;

    bool arm() override;
    bool disarm() override;

    bool takeoff(double altitudeMeters) override;
    bool goToLocation(LocationTarget target) override;
    bool moveByVelocity(VelocityCommand command, double durationSeconds) override;
    bool flyMission(const std::vector<MissionWaypoint>& waypoints) override;

    bool holdPosition() override;
    bool land() override;
    bool returnToHome() override;
    bool emergencyStop() override;

    bool setMaxSpeed(double metersPerSecond) override;
    bool setReturnHomeAltitude(double altitudeMeters) override;

    int batteryLife() const override;
    CurrentLocation currentLocation() const override;
    Attitude attitude() const override;
    DroneStatus status() const override;
    bool isConnected() const override;

private:
    bool connected_ = false;
    bool armed_ = false;
    int batteryPercent_ = 78;

    double maxSpeedMetersPerSecond_ = 5.0;
    double returnHomeAltitudeMeters_ = 20.0;

    CurrentLocation homeLocation_{
        47.62901,
        -122.31455,
        0.0,
        100.0
    };

    CurrentLocation currentLocation_ = homeLocation_;
    Attitude attitude_{};
};
