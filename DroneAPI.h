/*
 * DroneAPI is the common boundary between the application and the code that
 * actually controls a drone. I keep the interface independent of MAVSDK so the
 * console can work with either the in-memory mock or the MAVSDK implementation.
 * The public commands use meters internally, even though the console accepts
 * several values in feet for convenience.
 */

#pragma once

#include <string>
#include <vector>

// The interface is deliberately small. New front ends can depend on this class
// without needing to know whether commands are going to a mock, SITL, or hardware.
class DroneAPI {
public:
    // A destination uses relative altitude because that is easier to reason about
    // from a takeoff point. Implementations can convert it when a library expects AMSL.
    struct LocationTarget {
        double latitude = 0.0;
        double longitude = 0.0;
        double relativeAltitudeMeters = 0.0;
    };

    // Telemetry keeps both altitude references because PX4 exposes both and each
    // is useful for a different kind of display or calculation.
    struct CurrentLocation {
        double latitude = 0.0;
        double longitude = 0.0;
        double relativeAltitudeMeters = 0.0;
        double absoluteAltitudeMeters = 0.0;
    };

    struct Attitude {
        double rollDegrees = 0.0;
        double pitchDegrees = 0.0;
        double yawDegrees = 0.0;
    };

    struct DroneStatus {
        bool connected = false;
        bool armed = false;
        bool healthy = false;
        bool inAir = false;
        int batteryPercent = -1;
        std::string flightMode;
        std::string statusMessage;
    };

    // The field names follow north/east/down terminology at the API boundary. The
    // MAVSDK controller currently maps the horizontal fields to body-frame motion.
    struct VelocityCommand {
        double northMetersPerSecond = 0.0;
        double eastMetersPerSecond = 0.0;
        double downMetersPerSecond = 0.0;
        double yawDegreesPerSecond = 0.0;
    };

    struct MissionWaypoint {
        LocationTarget target;
        double holdSeconds = 0.0;
    };

    virtual ~DroneAPI() = default;

    // Connection management
    virtual bool connectToDrone(int droneID = 1) = 0;
    virtual bool connectToDrone(const std::string& connectionUrl) = 0;

    // Safety and power states
    virtual bool arm() = 0;
    virtual bool disarm() = 0;

    // Flight actions
    virtual bool takeoff(double altitudeMeters) = 0;
    virtual bool goToLocation(LocationTarget target) = 0;
    virtual bool moveByVelocity(VelocityCommand command, double durationSeconds) = 0;
    virtual bool flyMission(const std::vector<MissionWaypoint>& waypoints) = 0;

    // Contingency and landing maneuvers
    virtual bool holdPosition() = 0;
    virtual bool land() = 0;
    virtual bool returnToHome() = 0;
    virtual bool emergencyStop() = 0;

    // Parameter configuration
    virtual bool setMaxSpeed(double metersPerSecond) = 0;
    virtual bool setReturnHomeAltitude(double altitudeMeters) = 0;

    // Telemetry and state data
    virtual int batteryLife() const = 0;
    virtual CurrentLocation currentLocation() const = 0;
    virtual Attitude attitude() const = 0;
    virtual DroneStatus status() const = 0;
    virtual bool isConnected() const = 0;
};
