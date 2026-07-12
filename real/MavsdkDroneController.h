/*
 * MavsdkDroneController is the DroneAPI implementation that communicates with
 * PX4 through MAVSDK. The public methods mirror the project-level interface,
 * while the private helpers handle connection discovery, plugin setup, command
 * result reporting, and telemetry-based waits.
 */

#pragma once

#include "../DroneAPI.h"
#include "DroneSafetyLimits.h"

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/param/param.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

// The controller owns MAVSDK plugin objects after discovery. The shared System
// pointer keeps the discovered autopilot alive for the lifetime of those plugins.
class MavsdkDroneController : public DroneAPI {
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
    bool connectToDrone(
        const std::string& connectionUrl,
        const std::string& connectionLabel
    );

    bool openConnection(const std::string& connectionUrl);
    bool discoverAutopilot(const std::string& connectionLabel);
    void initializePlugins();

    bool ready(const std::string& operationName) const;

    bool requireArmed(
        const std::string& operationName
    ) const;

    bool runActionCommand(
        mavsdk::Action::Result result,
        const std::string& commandName
    ) const;

    bool ensureArmed();

    bool setTakeoffAltitude(double altitudeMeters);

    void printTargetLocation(
        LocationTarget target,
        float targetAbsoluteAltitudeMeters
    ) const;

    bool waitUntil(
        std::function<bool()> condition,
        std::chrono::seconds timeout
    ) const;

    bool waitUntilHealthy(std::chrono::seconds timeout) const;

    bool waitUntilRelativeAltitude(
        double targetAltitudeMeters,
        std::chrono::seconds timeout
    ) const;

    bool waitUntilNearTarget(
        LocationTarget target,
        float targetAbsoluteAltitudeMeters,
        double horizontalToleranceMeters,
        double verticalToleranceMeters,
        std::chrono::seconds timeout
    ) const;

    std::chrono::seconds estimateGoToLocationTimeout(
        double horizontalDistanceMeters,
        double verticalDistanceMeters
    ) const;

    bool waitUntilLandedOrDisarmed(std::chrono::seconds timeout) const;

private:
    mavsdk::Mavsdk mavsdk_{
        mavsdk::Mavsdk::Configuration{
            mavsdk::ComponentType::GroundStation
        }
    };

    std::shared_ptr<mavsdk::System> system_ = nullptr;

    std::unique_ptr<mavsdk::Action> action_ = nullptr;
    std::unique_ptr<mavsdk::Telemetry> telemetry_ = nullptr;
    std::unique_ptr<mavsdk::Offboard> offboard_ = nullptr;
    std::unique_ptr<mavsdk::Param> param_ = nullptr;
    
    // These limits are checked in addition to any protections configured in PX4.
    SafetyLimits safetyLimits_{};

    bool connected_ = false;
    bool connectionOpen_ = false;
    std::string activeConnectionUrl_;
    
};
