/*
 * This file contains the MAVSDK-backed flight implementation. Commands are
 * validated before they are sent, and longer actions wait for telemetry to
 * confirm that the vehicle actually reached the expected state. That makes a
 * successful return value more meaningful than simply knowing PX4 accepted a
 * request.
 */

#include "MavsdkDroneController.h"

#include "DroneCommandUtils.h"
#include "MavsdkTelemetryUtils.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

namespace {
    double metersToFeetForDisplay(double meters) {
        return meters / 0.3048;
    }
}

// Both public connection methods arrive here after choosing a URL and a label. This
// keeps connection reuse, discovery, and plugin initialization in one path.
bool MavsdkDroneController::connectToDrone(int droneID) {
    if (!DroneCommandUtils::validDroneID(droneID)) {
        std::cerr << "Drone ID must be 1 or higher.\n";
        return false;
    }

    std::string connectionUrl =
        DroneCommandUtils::connectionUrlForDrone(droneID);

    return connectToDrone(
        connectionUrl,
        "drone ID " + std::to_string(droneID)
    );
}

bool MavsdkDroneController::connectToDrone(const std::string& connectionUrl) {
    return connectToDrone(connectionUrl, "custom connection URL");
}

bool MavsdkDroneController::arm() {
    if (!ready("arm")) {
        return false;
    }

    if (!waitUntilHealthy(std::chrono::seconds(30))) {
        std::cerr << "Drone is not healthy enough to arm.\n";
        return false;
    }

    return runActionCommand(action_->arm(), "Arming");
}

bool MavsdkDroneController::disarm() {
    if (!ready("disarm")) {
        return false;
    }

    if (!telemetry_->armed()) {
        std::cout << "Drone is already disarmed.\n";
        return true;
    }

    return runActionCommand(action_->disarm(), "Disarm");
}

bool MavsdkDroneController::takeoff(double altitudeMeters) {
    if (!ready("take off")) {
        return false;
    }

    if (!DroneCommandUtils::validTakeoffAltitude(altitudeMeters)) {
        return false;
    }
    
    if (altitudeMeters < safetyLimits_.minRelativeAltitudeMeters ||
		altitudeMeters > safetyLimits_.maxRelativeAltitudeMeters) {
		std::cerr << "Takeoff altitude must be between "
				  << metersToFeetForDisplay(safetyLimits_.minRelativeAltitudeMeters)
				  << " and "
				  << metersToFeetForDisplay(safetyLimits_.maxRelativeAltitudeMeters)
				  << " feet for real-drone safety.\n";
		return false;
	}

    if (!setTakeoffAltitude(altitudeMeters)) {
        return false;
    }

    if (!ensureArmed()) {
        return false;
    }

    if (!runActionCommand(action_->takeoff(), "Takeoff")) {
        return false;
    }

    std::cout << "Takeoff command accepted. Target altitude: "
    		  << metersToFeetForDisplay(altitudeMeters)
    		  << " feet.\n";

    return waitUntilRelativeAltitude(
        altitudeMeters,
        std::chrono::seconds(1800)
    );
}

// MAVSDK expects an absolute target altitude, while DroneAPI exposes a relative one.
// This method validates the request, performs that conversion, and waits for arrival.
bool MavsdkDroneController::goToLocation(LocationTarget target) {
    if (!ready("go to location")) {
        return false;
    }

    if (!requireArmed("go to location")) {
        return false;
    }

    if (!DroneCommandUtils::validLocationTarget(target)) {
        return false;
    }

    mavsdk::Telemetry::Position currentPosition = telemetry_->position();

    if (!MavsdkTelemetryUtils::validTelemetryPosition(currentPosition)) {
        std::cerr << "Cannot go to location. Current GPS position is invalid.\n";
        return false;
    }

    double distanceToTargetMeters =
        DroneCommandUtils::distanceMeters(
            currentPosition.latitude_deg,
            currentPosition.longitude_deg,
            target.latitude,
            target.longitude
        );

    if (distanceToTargetMeters > safetyLimits_.maxHorizontalDistanceMeters) {
		std::cerr << "Target is too far away for real-drone safety. Distance: "
				  << metersToFeetForDisplay(distanceToTargetMeters)
				  << " feet. Limit: "
				  << metersToFeetForDisplay(safetyLimits_.maxHorizontalDistanceMeters)
				  << " feet.\n";
		return false;
	}

    if (target.relativeAltitudeMeters < safetyLimits_.minRelativeAltitudeMeters ||
        target.relativeAltitudeMeters > safetyLimits_.maxRelativeAltitudeMeters) {
        std::cerr << "Target altitude must be between "
        		  << metersToFeetForDisplay(safetyLimits_.minRelativeAltitudeMeters)
        		  << " and "
        		  << metersToFeetForDisplay(safetyLimits_.maxRelativeAltitudeMeters)
        		  << " feet.\n";
        return false;
    }

    /*
        DroneAPI uses relative altitude for public commands.
        MAVSDK goto_location expects absolute altitude above mean sea level.

        We estimate the home or takeoff reference as:

            home absolute altitude = current absolute altitude - current relative altitude

        Then we convert the requested relative target altitude into the absolute altitude
        value expected by MAVSDK.
    */
    float homeAbsoluteAltitudeMeters =
        currentPosition.absolute_altitude_m -
        currentPosition.relative_altitude_m;

    float targetAbsoluteAltitudeMeters =
        homeAbsoluteAltitudeMeters +
        static_cast<float>(target.relativeAltitudeMeters);

    double startingHorizontalDistance = distanceToTargetMeters;

    double startingVerticalDistance =
        std::fabs(
            static_cast<double>(currentPosition.absolute_altitude_m) -
            static_cast<double>(targetAbsoluteAltitudeMeters)
        );

    std::chrono::seconds timeout =
        estimateGoToLocationTimeout(
            startingHorizontalDistance,
            startingVerticalDistance
        );

    /*
        Yaw controls which direction the drone faces.
        0 degrees means north.
    */
    float yawDegrees = 0.0f;

    mavsdk::Action::Result result =
        action_->goto_location(
            target.latitude,
            target.longitude,
            targetAbsoluteAltitudeMeters,
            yawDegrees
        );

    if (!runActionCommand(result, "Go-to-location")) {
        return false;
    }

    printTargetLocation(target, targetAbsoluteAltitudeMeters);

    std::cout << "Estimated travel timeout: "
              << timeout.count()
              << " seconds.\n";

    return waitUntilNearTarget(
        target,
        targetAbsoluteAltitudeMeters,
        5.0,
        3.0,
        timeout
    );
}

// Offboard velocity is streamed repeatedly because PX4 expects a continuing setpoint
// rather than one command that remains active on its own.
bool MavsdkDroneController::moveByVelocity(
    VelocityCommand command,
    double durationSeconds
) {
    if (!ready("move by velocity")) {
        return false;
    }

    if (!requireArmed("move by velocity")) {
        return false;
    }

    if (!std::isfinite(durationSeconds) || durationSeconds <= 0.0) {
        std::cerr << "Duration must be positive.\n";
        return false;
    }

    if (!std::isfinite(command.northMetersPerSecond) ||
        !std::isfinite(command.eastMetersPerSecond) ||
        !std::isfinite(command.downMetersPerSecond) ||
        !std::isfinite(command.yawDegreesPerSecond)) {
        std::cerr << "Velocity command values must be real numbers.\n";
        return false;
    }

    if (durationSeconds > safetyLimits_.maxVelocityDurationSeconds) {
        std::cerr << "Velocity duration is too long. Requested: "
                  << durationSeconds
                  << " seconds. Limit: "
                  << safetyLimits_.maxVelocityDurationSeconds
                  << " seconds.\n";
        return false;
    }

    if (std::fabs(command.downMetersPerSecond) >
        safetyLimits_.maxVerticalSpeedMetersPerSecond) {
        std::cerr << "Vertical velocity is too high. Requested: "
        		  << metersToFeetForDisplay(std::fabs(command.downMetersPerSecond))
        		  << " ft/s. Limit: "
        		  << metersToFeetForDisplay(safetyLimits_.maxVerticalSpeedMetersPerSecond)
        		  << " ft/s.\n";
        return false;
    }

    mavsdk::Telemetry::Position currentPosition = telemetry_->position();

    if (!MavsdkTelemetryUtils::validTelemetryPosition(currentPosition)) {
        std::cerr << "Cannot move by velocity. Current GPS position is invalid.\n";
        return false;
    }

    double predictedRelativeAltitude =
        currentPosition.relative_altitude_m -
        command.downMetersPerSecond * durationSeconds;

    if (predictedRelativeAltitude < safetyLimits_.minRelativeAltitudeMeters) {
        std::cerr << "Velocity command would go below minimum safe altitude. "
                  << "Predicted altitude: "
                  << metersToFeetForDisplay(predictedRelativeAltitude)
                  << " feet. Minimum: "
                  << metersToFeetForDisplay(safetyLimits_.minRelativeAltitudeMeters)
                  << " feet.\n";
        return false;
    }

    if (predictedRelativeAltitude > safetyLimits_.maxRelativeAltitudeMeters) {
        std::cerr << "Velocity command would exceed maximum safe altitude. "
                  << "Predicted altitude: "
                  << metersToFeetForDisplay(predictedRelativeAltitude)
                  << " feet. Maximum: "
                  << metersToFeetForDisplay(safetyLimits_.maxRelativeAltitudeMeters)
                  << " feet.\n";
        return false;
    }

    double horizontalSpeed =
        std::hypot(
            command.northMetersPerSecond,
            command.eastMetersPerSecond
        );

    if (horizontalSpeed > safetyLimits_.maxVelocityMetersPerSecond) {
        std::cerr << "Horizontal velocity is too high. Requested: "
                  << metersToFeetForDisplay(horizontalSpeed)
                  << " ft/s. Limit: "
                  << metersToFeetForDisplay(safetyLimits_.maxVelocityMetersPerSecond)
                  << " ft/s.\n";
        return false;
    }

    if (std::fabs(command.yawDegreesPerSecond) >
        safetyLimits_.maxYawSpeedDegreesPerSecond) {
        std::cerr << "Yaw speed is too high. Requested: "
                  << command.yawDegreesPerSecond
                  << " deg/s. Limit: "
                  << safetyLimits_.maxYawSpeedDegreesPerSecond
                  << " deg/s.\n";
        return false;
    }

    /*
        MAVSDK body-frame velocity uses:
            forward_m_s
            right_m_s
            down_m_s
            yawspeed_deg_s

        This maps the existing DroneAPI fields as:
            northMetersPerSecond -> forward
            eastMetersPerSecond  -> right

        So this is body-frame movement, not true north/east movement.
    */
    mavsdk::Offboard::VelocityBodyYawspeed velocity{};
    velocity.forward_m_s = static_cast<float>(command.northMetersPerSecond);
    velocity.right_m_s = static_cast<float>(command.eastMetersPerSecond);
    velocity.down_m_s = static_cast<float>(command.downMetersPerSecond);
    velocity.yawspeed_deg_s = static_cast<float>(command.yawDegreesPerSecond);

    mavsdk::Offboard::Result result =
        offboard_->set_velocity_body(velocity);

    if (result != mavsdk::Offboard::Result::Success) {
        std::cerr << "Set initial offboard body velocity failed: "
                  << result << '\n';
        return false;
    }

    result = offboard_->start();

    if (result != mavsdk::Offboard::Result::Success) {
        std::cerr << "Offboard start failed: "
                  << result << '\n';
        return false;
    }

    std::cout << "Offboard body velocity control started.\n";

    auto start = std::chrono::steady_clock::now();

    while (std::chrono::steady_clock::now() - start <
           std::chrono::duration<double>(durationSeconds)) {
        result = offboard_->set_velocity_body(velocity);

        if (result != mavsdk::Offboard::Result::Success) {
            std::cerr << "Set offboard body velocity failed: "
                      << result << '\n';

            mavsdk::Offboard::VelocityBodyYawspeed stopVelocity{};
            offboard_->set_velocity_body(stopVelocity);
            offboard_->stop();

            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    mavsdk::Offboard::VelocityBodyYawspeed stopVelocity{};
    result = offboard_->set_velocity_body(stopVelocity);

    if (result != mavsdk::Offboard::Result::Success) {
        std::cerr << "Set stop velocity failed: "
                  << result << '\n';
    }

    result = offboard_->stop();

    if (result != mavsdk::Offboard::Result::Success) {
        std::cerr << "Offboard stop failed: "
                  << result << '\n';
        return false;
    }

    std::cout << "Offboard body velocity control completed.\n";
    return true;
}

bool MavsdkDroneController::flyMission(
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

        if (waypoint.holdSeconds > 0.0) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(
                    static_cast<int>(waypoint.holdSeconds * 1000)
                )
            );
        }
    }

    std::cout << "Mission completed.\n";
    return true;
}

bool MavsdkDroneController::holdPosition() {
    if (!ready("hold position")) {
        return false;
    }

    if (!requireArmed("hold position")) {
        return false;
    }

    return runActionCommand(action_->hold(), "Hold position");
}

bool MavsdkDroneController::land() {
    if (!ready("land")) {
        return false;
    }

    if (!telemetry_->armed()) {
        std::cout << "Drone is already disarmed. Treating as landed.\n";
        return true;
    }

    auto landedState = telemetry_->landed_state();

    if (landedState == mavsdk::Telemetry::LandedState::OnGround) {
        std::cout << "Drone is already on the ground.\n";
        return true;
    }

    if (!runActionCommand(action_->land(), "Landing")) {
        return false;
    }

    std::cout << "Landing command accepted.\n";

    return waitUntilLandedOrDisarmed(std::chrono::seconds(1800));
}

bool MavsdkDroneController::returnToHome() {
    if (!ready("return home")) {
        return false;
    }

    if (!requireArmed("return home")) {
        return false;
    }

    auto landedState = telemetry_->landed_state();

    if (landedState == mavsdk::Telemetry::LandedState::OnGround) {
        std::cerr << "Cannot return home. Drone is already on the ground.\n";
        return false;
    }

    if (landedState != mavsdk::Telemetry::LandedState::InAir) {
        std::cerr << "Cannot return home. Drone is not confirmed to be in the air.\n";
        return false;
    }

    return runActionCommand(
        action_->return_to_launch(),
        "Return-to-home"
    );
}

bool MavsdkDroneController::emergencyStop() {
    std::cerr << "Emergency stop is not implemented yet for the MAVSDK controller.\n";
    std::cerr << "For a real drone, this should be implemented very carefully.\n";

    return false;
}

bool MavsdkDroneController::setMaxSpeed(double metersPerSecond) {
    if (!connected_ || !system_ || !param_) {
        std::cerr << "Cannot set max speed. Drone is not connected.\n";
        return false;
    }

    if (!std::isfinite(metersPerSecond) || metersPerSecond <= 0.0) {
        std::cerr << "Max speed must be positive.\n";
        return false;
    }

    if (metersPerSecond > 20.0) {
        std::cerr << "Max speed must be "
        		  << metersToFeetForDisplay(20.0)
        		  << " ft/s or lower for PX4 MPC_XY_VEL_MAX.\n";
        return false;
    }

    mavsdk::Param::Result result =
        param_->set_param_float(
            "MPC_XY_VEL_MAX",
            static_cast<float>(metersPerSecond)
        );

    if (result != mavsdk::Param::Result::Success) {
        std::cerr << "Failed to set PX4 max horizontal velocity: "
                  << result << '\n';
        return false;
    }

    std::cout << "PX4 max horizontal velocity set to "
              << metersToFeetForDisplay(metersPerSecond)
              << " ft/s.\n";

    return true;
}

bool MavsdkDroneController::setReturnHomeAltitude(double altitudeMeters) {
    if (!std::isfinite(altitudeMeters) || altitudeMeters <= 0.0) {
        std::cerr << "Return-home altitude must be positive.\n";
        return false;
    }

    std::cerr << "Return-home altitude is not implemented yet.\n";
    std::cerr << "This usually requires setting PX4 parameters.\n";

    return false;
}

int MavsdkDroneController::batteryLife() const {
    if (!telemetry_) {
        return -1;
    }

    mavsdk::Telemetry::Battery battery = telemetry_->battery();
    float percent = battery.remaining_percent;

    if (!std::isfinite(percent)) {
        return -1;
    }

    /*
        Some MAVLink or MAVSDK setups expose battery remaining as 0.0 to 1.0.
        Others expose it as 0 to 100.

        This keeps the public API stable by returning an integer from 0 to 100.
    */
    if (percent <= 1.0f) {
        percent *= 100.0f;
    }

    return std::clamp(
        static_cast<int>(std::round(percent)),
        0,
        100
    );
}

DroneAPI::CurrentLocation MavsdkDroneController::currentLocation() const {
    if (!telemetry_) {
        return MavsdkTelemetryUtils::invalidCurrentLocation();
    }

    mavsdk::Telemetry::Position position = telemetry_->position();

    if (!MavsdkTelemetryUtils::validTelemetryPosition(position)) {
        return MavsdkTelemetryUtils::invalidCurrentLocation();
    }

    return MavsdkTelemetryUtils::positionToCurrentLocation(position);
}

DroneAPI::Attitude MavsdkDroneController::attitude() const {
    if (!telemetry_) {
        return {};
    }

    mavsdk::Telemetry::EulerAngle euler = telemetry_->attitude_euler();

    return {
        euler.roll_deg,
        euler.pitch_deg,
        euler.yaw_deg
    };
}

DroneAPI::DroneStatus MavsdkDroneController::status() const {
    DroneStatus status;

    status.connected = connected_;
    status.batteryPercent = batteryLife();

    if (!telemetry_) {
        status.statusMessage = "Telemetry unavailable";
        return status;
    }

    status.armed = telemetry_->armed();
    status.healthy = telemetry_->health_all_ok();

    auto landedState = telemetry_->landed_state();

    status.inAir =
        landedState == mavsdk::Telemetry::LandedState::InAir;

    status.statusMessage = connected_
        ? "MAVSDK drone connected"
        : "MAVSDK drone disconnected";

    return status;
}

bool MavsdkDroneController::isConnected() const {
    return connected_;
}

bool MavsdkDroneController::connectToDrone(
    const std::string& connectionUrl,
    const std::string& connectionLabel
) {
    if (connected_) {
        std::cout << "Drone is already connected.\n";
        return true;
    }

    if (!openConnection(connectionUrl)) {
        return false;
    }

    if (!discoverAutopilot(connectionLabel)) {
        return false;
    }

    initializePlugins();

    connected_ = true;

    std::cout << "Connected using " << connectionLabel << ".\n";
    return true;
}

bool MavsdkDroneController::openConnection(const std::string& connectionUrl) {
    if (connectionOpen_) {
        if (activeConnectionUrl_ == connectionUrl) {
            std::cout << "Connection already opened on "
                      << connectionUrl
                      << ". Retrying autopilot discovery...\n";
            return true;
        }

        std::cerr << "A MAVSDK connection is already open on "
                  << activeConnectionUrl_
                  << ". Restart the app to use a different connection URL.\n";
        return false;
    }

    mavsdk::ConnectionResult result =
        mavsdk_.add_any_connection(connectionUrl);

    if (result != mavsdk::ConnectionResult::Success) {
        std::cerr << "Connection failed on "
                  << connectionUrl << ": "
                  << result << '\n';
        return false;
    }

    connectionOpen_ = true;
    activeConnectionUrl_ = connectionUrl;

    std::cout << "Waiting for drone on "
              << connectionUrl << "...\n";

    return true;
}

bool MavsdkDroneController::discoverAutopilot(
    const std::string& connectionLabel
) {
    auto systemOptional = mavsdk_.first_autopilot(10.0);

    if (!systemOptional.has_value()) {
        std::cerr << "No autopilot discovered for "
                  << connectionLabel << ".\n";
        return false;
    }

    system_ = systemOptional.value();
    return true;
}

void MavsdkDroneController::initializePlugins() {
    action_ = std::make_unique<mavsdk::Action>(system_);
    telemetry_ = std::make_unique<mavsdk::Telemetry>(system_);
    offboard_ = std::make_unique<mavsdk::Offboard>(system_);
    param_ = std::make_unique<mavsdk::Param>(system_);
}

bool MavsdkDroneController::ready(const std::string& operationName) const {
    if (!connected_ || !system_ || !action_ || !telemetry_ || !offboard_) {
        std::cerr << "Cannot " << operationName
                  << ". Drone is not connected.\n";
        return false;
    }

    return true;
}

bool MavsdkDroneController::requireArmed(
    const std::string& operationName
) const {
    if (!telemetry_->armed()) {
        std::cerr << "Cannot " << operationName
                  << ". Drone is not armed.\n";
        return false;
    }

    return true;
}

bool MavsdkDroneController::runActionCommand(
    mavsdk::Action::Result result,
    const std::string& commandName
) const {
    if (result != mavsdk::Action::Result::Success) {
        std::cerr << commandName << " failed: "
                  << result << '\n';
        return false;
    }

    std::cout << commandName << " succeeded.\n";
    return true;
}

bool MavsdkDroneController::ensureArmed() {
    if (telemetry_->armed()) {
        return true;
    }

    return arm();
}

bool MavsdkDroneController::setTakeoffAltitude(double altitudeMeters) {
    mavsdk::Action::Result result =
        action_->set_takeoff_altitude(
            static_cast<float>(altitudeMeters)
        );

    return runActionCommand(result, "Set takeoff altitude");
}

void MavsdkDroneController::printTargetLocation(
    LocationTarget target,
    float targetAbsoluteAltitudeMeters
) const {
    std::cout << "Moving to location: "
              << target.latitude << ", "
              << target.longitude << ", "
              << metersToFeetForDisplay(target.relativeAltitudeMeters)
              << " feet relative altitude"
              << " using "
              << metersToFeetForDisplay(targetAbsoluteAltitudeMeters)
              << " feet AMSL for MAVSDK.\n";
}

// Longer commands use this polling helper so timeout behavior stays consistent and
// no action method has to duplicate the same sleep loop.
bool MavsdkDroneController::waitUntil(
    std::function<bool()> condition,
    std::chrono::seconds timeout
) const {
    auto start = std::chrono::steady_clock::now();

    while (std::chrono::steady_clock::now() - start < timeout) {
        if (condition()) {
            return true;
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return false;
}

bool MavsdkDroneController::waitUntilHealthy(
    std::chrono::seconds timeout
) const {
    bool healthy = waitUntil(
        [this]() {
            return telemetry_->health_all_ok();
        },
        timeout
    );

    if (!healthy) {
        return false;
    }

    std::cout << "Drone health checks passed.\n";
    return true;
}

bool MavsdkDroneController::waitUntilRelativeAltitude(
    double targetAltitudeMeters,
    std::chrono::seconds timeout
) const {
    bool reachedAltitude = waitUntil(
        [this, targetAltitudeMeters]() {
            mavsdk::Telemetry::Position position = telemetry_->position();

            if (!MavsdkTelemetryUtils::validTelemetryPosition(position)) {
                return false;
            }

            return position.relative_altitude_m >=
                   targetAltitudeMeters - 0.5;
        },
        timeout
    );

    if (!reachedAltitude) {
        std::cerr << "Timed out waiting for takeoff altitude.\n";
        return false;
    }

    std::cout << "Reached takeoff altitude.\n";
    return true;
}

// The timeout is based on the farther of the horizontal or vertical trip, with a
// generous buffer for acceleration, settling, and simulator delays.
std::chrono::seconds MavsdkDroneController::estimateGoToLocationTimeout(
    double horizontalDistanceMeters,
    double verticalDistanceMeters
) const {
    /*
        PX4 SITL usually moves at a few meters per second in these tests.
        Use conservative values so longer trips do not time out too early.
    */
    constexpr double expectedHorizontalSpeedMetersPerSecond = 4.0;
    constexpr double expectedVerticalSpeedMetersPerSecond = 1.0;

    double horizontalSeconds =
        horizontalDistanceMeters / expectedHorizontalSpeedMetersPerSecond;

    double verticalSeconds =
        verticalDistanceMeters / expectedVerticalSpeedMetersPerSecond;

    double estimatedSeconds =
        std::max(horizontalSeconds, verticalSeconds);

    /*
        Add buffer time for acceleration, slowing near the target,
        simulator delays, and GPS or altitude settling.
    */
    estimatedSeconds += 60.0;

    /*
        Minimum timeout for short trips.
    */
    estimatedSeconds = std::max(estimatedSeconds, 60.0);

    /*
        Maximum timeout for simulation.
        This caps the wait at 30 minutes.
    */
    estimatedSeconds = std::min(estimatedSeconds, 1800.0);

    return std::chrono::seconds(
        static_cast<int>(std::ceil(estimatedSeconds))
    );
}

bool MavsdkDroneController::waitUntilNearTarget(
    LocationTarget target,
    float targetAbsoluteAltitudeMeters,
    double horizontalToleranceMeters,
    double verticalToleranceMeters,
    std::chrono::seconds timeout
) const {
    bool reachedTarget = waitUntil(
        [this,
         target,
         targetAbsoluteAltitudeMeters,
         horizontalToleranceMeters,
         verticalToleranceMeters]() {
            mavsdk::Telemetry::Position currentPosition =
                telemetry_->position();

            if (!MavsdkTelemetryUtils::validTelemetryPosition(
                    currentPosition
                )) {
                return false;
            }

            double horizontalDistance =
                DroneCommandUtils::distanceMeters(
                    currentPosition.latitude_deg,
                    currentPosition.longitude_deg,
                    target.latitude,
                    target.longitude
                );

            double verticalDistance =
                std::fabs(
                    static_cast<double>(
                        currentPosition.absolute_altitude_m
                    ) -
                    static_cast<double>(targetAbsoluteAltitudeMeters)
                );

            std::cout << "Distance to target: "
            		  << metersToFeetForDisplay(horizontalDistance)
            		  << " feet, vertical difference: "
            		  << metersToFeetForDisplay(verticalDistance)
            		  << " feet\n";

            return horizontalDistance <= horizontalToleranceMeters &&
                   verticalDistance <= verticalToleranceMeters;
        },
        timeout
    );

    if (!reachedTarget) {
        std::cerr << "Timed out waiting to reach target location.\n";
        return false;
    }

    std::cout << "Reached target location.\n";
    return true;
}

bool MavsdkDroneController::waitUntilLandedOrDisarmed(
    std::chrono::seconds timeout
) const {
    bool landedOrDisarmed = waitUntil(
        [this]() {
            if (!telemetry_->armed()) {
                return true;
            }

            mavsdk::Telemetry::Position position = telemetry_->position();

            if (MavsdkTelemetryUtils::validTelemetryPosition(position) &&
                std::fabs(position.relative_altitude_m) <= 0.3f) {
                return true;
            }

            auto landedState = telemetry_->landed_state();

            return landedState ==
                   mavsdk::Telemetry::LandedState::OnGround;
        },
        timeout
    );

    if (!landedOrDisarmed) {
        std::cerr << "Landing command sent, but landing was not confirmed in time.\n";
        return false;
    }

    std::cout << "Landing confirmed.\n";
    return true;
}
