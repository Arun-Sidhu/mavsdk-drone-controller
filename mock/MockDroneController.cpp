/*
 * The mock controller stores a simple version of the drone state in memory.
 * Each successful command updates that state immediately, which keeps the mock
 * predictable and makes it easy to test the interface. Battery drain, travel
 * time, wind, GPS noise, and most other physical behavior are intentionally out
 * of scope here.
 */

#include "MockDroneController.h"

#include <cmath>
#include <iostream>

bool MockDroneController::connectToDrone(int droneID) {
    if (droneID < 1) {
        std::cerr << "Drone ID must be 1 or higher.\n";
        return false;
    }

    connected_ = true;
    std::cout << "Mock connected to drone ID " << droneID << ".\n";
    return true;
}

bool MockDroneController::connectToDrone(const std::string& connectionUrl) {
    if (connectionUrl.empty()) {
        std::cerr << "Connection URL cannot be empty.\n";
        return false;
    }

    connected_ = true;
    std::cout << "Mock connected using " << connectionUrl << ".\n";
    return true;
}

bool MockDroneController::arm() {
    if (!connected_) {
        std::cerr << "Cannot arm. Mock drone is not connected.\n";
        return false;
    }

    armed_ = true;
    std::cout << "Mock drone armed.\n";
    return true;
}

bool MockDroneController::disarm() {
    if (!connected_) {
        std::cerr << "Cannot disarm. Mock drone is not connected.\n";
        return false;
    }

    if (currentLocation_.relativeAltitudeMeters > 0.3) {
        std::cerr << "Cannot disarm while mock drone is in the air.\n";
        return false;
    }

    armed_ = false;
    std::cout << "Mock drone disarmed.\n";
    return true;
}

// Mock actions complete immediately. The goal is to test state transitions and
// calling code, not to imitate the timing of a real takeoff.
bool MockDroneController::takeoff(double altitudeMeters) {
    if (!connected_) {
        std::cerr << "Cannot take off. Mock drone is not connected.\n";
        return false;
    }

    if (!armed_ && !arm()) {
        return false;
    }

    if (!std::isfinite(altitudeMeters) || altitudeMeters <= 0.0) {
        std::cerr << "Takeoff altitude must be a positive real number.\n";
        return false;
    }

    currentLocation_.relativeAltitudeMeters = altitudeMeters;
    currentLocation_.absoluteAltitudeMeters =
        homeLocation_.absoluteAltitudeMeters + altitudeMeters;

    std::cout << "Mock drone took off to "
              << altitudeMeters
              << " meters relative altitude.\n";

    return true;
}

bool MockDroneController::goToLocation(LocationTarget target) {
    if (!connected_) {
        std::cerr << "Cannot go to location. Mock drone is not connected.\n";
        return false;
    }

    if (!armed_) {
        std::cerr << "Cannot go to location. Mock drone is not armed.\n";
        return false;
    }

    if (!std::isfinite(target.latitude) ||
        target.latitude < -90.0 ||
        target.latitude > 90.0) {
        std::cerr << "Invalid latitude.\n";
        return false;
    }

    if (!std::isfinite(target.longitude) ||
        target.longitude < -180.0 ||
        target.longitude > 180.0) {
        std::cerr << "Invalid longitude.\n";
        return false;
    }

    if (!std::isfinite(target.relativeAltitudeMeters) ||
        target.relativeAltitudeMeters < 1.0) {
        std::cerr << "Relative altitude should be at least 1 meter.\n";
        return false;
    }

    currentLocation_.latitude = target.latitude;
    currentLocation_.longitude = target.longitude;
    currentLocation_.relativeAltitudeMeters = target.relativeAltitudeMeters;
    currentLocation_.absoluteAltitudeMeters =
        homeLocation_.absoluteAltitudeMeters + target.relativeAltitudeMeters;

    std::cout << "Mock drone reached target location.\n";
    return true;
}

// The lightweight mock currently applies vertical and yaw changes only. Horizontal
// position changes would require a coordinate model that is outside this smoke test.
bool MockDroneController::moveByVelocity(
    VelocityCommand command,
    double durationSeconds
) {
    if (!connected_) {
        std::cerr << "Cannot move. Mock drone is not connected.\n";
        return false;
    }

    if (!armed_) {
        std::cerr << "Cannot move. Mock drone is not armed.\n";
        return false;
    }

    if (!std::isfinite(durationSeconds) || durationSeconds <= 0.0) {
        std::cerr << "Duration must be positive.\n";
        return false;
    }

    currentLocation_.relativeAltitudeMeters -=
        command.downMetersPerSecond * durationSeconds;

    if (currentLocation_.relativeAltitudeMeters < 0.0) {
        currentLocation_.relativeAltitudeMeters = 0.0;
    }

    currentLocation_.absoluteAltitudeMeters =
        homeLocation_.absoluteAltitudeMeters +
        currentLocation_.relativeAltitudeMeters;

    attitude_.yawDegrees += command.yawDegreesPerSecond * durationSeconds;

    std::cout << "Mock velocity command completed.\n";
    return true;
}

bool MockDroneController::flyMission(
    const std::vector<MissionWaypoint>& waypoints
) {
    if (waypoints.empty()) {
        std::cerr << "Mission must contain at least one waypoint.\n";
        return false;
    }

    for (const MissionWaypoint& waypoint : waypoints) {
        if (!goToLocation(waypoint.target)) {
            return false;
        }

        std::cout << "Holding for "
                  << waypoint.holdSeconds
                  << " seconds.\n";
    }

    std::cout << "Mock mission completed.\n";
    return true;
}

bool MockDroneController::holdPosition() {
    if (!connected_) {
        std::cerr << "Cannot hold position. Mock drone is not connected.\n";
        return false;
    }

    if (!armed_) {
        std::cerr << "Cannot hold position. Mock drone is not armed.\n";
        return false;
    }

    std::cout << "Mock drone is holding position.\n";
    return true;
}

bool MockDroneController::land() {
    if (!connected_) {
        std::cerr << "Cannot land. Mock drone is not connected.\n";
        return false;
    }

    currentLocation_.relativeAltitudeMeters = 0.0;
    currentLocation_.absoluteAltitudeMeters =
        homeLocation_.absoluteAltitudeMeters;

    armed_ = false;

    std::cout << "Mock drone landed and disarmed.\n";
    return true;
}

bool MockDroneController::returnToHome() {
    if (!connected_) {
        std::cerr << "Cannot return home. Mock drone is not connected.\n";
        return false;
    }

    if (!armed_) {
        std::cerr << "Cannot return home. Mock drone is not armed.\n";
        return false;
    }

    currentLocation_.latitude = homeLocation_.latitude;
    currentLocation_.longitude = homeLocation_.longitude;
    currentLocation_.relativeAltitudeMeters = returnHomeAltitudeMeters_;
    currentLocation_.absoluteAltitudeMeters =
        homeLocation_.absoluteAltitudeMeters + returnHomeAltitudeMeters_;

    std::cout << "Mock drone returned above home location.\n";
    return true;
}

bool MockDroneController::emergencyStop() {
    if (!connected_) {
        std::cerr << "Cannot emergency stop. Mock drone is not connected.\n";
        return false;
    }

    currentLocation_.relativeAltitudeMeters = 0.0;
    currentLocation_.absoluteAltitudeMeters =
        homeLocation_.absoluteAltitudeMeters;

    armed_ = false;

    std::cout << "Mock emergency stop completed.\n";
    return true;
}

bool MockDroneController::setMaxSpeed(double metersPerSecond) {
    if (!std::isfinite(metersPerSecond) || metersPerSecond <= 0.0) {
        std::cerr << "Max speed must be positive.\n";
        return false;
    }

    maxSpeedMetersPerSecond_ = metersPerSecond;

    std::cout << "Mock max speed set to "
              << maxSpeedMetersPerSecond_
              << " m/s.\n";

    return true;
}

bool MockDroneController::setReturnHomeAltitude(double altitudeMeters) {
    if (!std::isfinite(altitudeMeters) || altitudeMeters <= 0.0) {
        std::cerr << "Return home altitude must be positive.\n";
        return false;
    }

    returnHomeAltitudeMeters_ = altitudeMeters;

    std::cout << "Mock return-home altitude set to "
              << returnHomeAltitudeMeters_
              << " meters.\n";

    return true;
}

int MockDroneController::batteryLife() const {
    if (!connected_) {
        return -1;
    }

    return batteryPercent_;
}

DroneAPI::CurrentLocation MockDroneController::currentLocation() const {
    return currentLocation_;
}

DroneAPI::Attitude MockDroneController::attitude() const {
    return attitude_;
}

// Build a fresh status object from the stored state so callers see the same shape
// of data they receive from the MAVSDK implementation.
DroneAPI::DroneStatus MockDroneController::status() const {
    DroneStatus status;

    status.connected = connected_;
    status.armed = armed_;
    status.healthy = connected_;
    status.inAir = currentLocation_.relativeAltitudeMeters > 0.3;
    status.batteryPercent = batteryLife();
    status.flightMode = status.inAir ? "Mock flight" : "Mock ground";
    status.statusMessage = connected_ ? "Mock drone ready" : "Mock drone disconnected";

    return status;
}

bool MockDroneController::isConnected() const {
    return connected_;
}
