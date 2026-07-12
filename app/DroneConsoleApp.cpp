/*
 * This file contains the interactive side of the project. It reads commands
 * from the terminal, performs a few early safety checks, and then delegates the
 * real work to DroneAPI. The controller still validates every flight command,
 * so the checks here are an extra layer rather than the only line of defense.
 * User-facing distances are mostly shown in feet, while DroneAPI stays metric.
 */

#include "DroneConsoleApp.h"
#include "ElevationService.h"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <vector>

namespace {
    // These helpers belong only to the console. Keeping them local avoids adding
    // display-oriented units and direction choices to the controller interface.
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kEarthRadiusMeters = 6378137.0;
    constexpr double kFeetToMeters = 0.3048;
    constexpr double kMaxDistanceFromBaseFeet = 1000.0;

    enum class RelativeDirection {
        West,
        Northwest,
        North,
        Northeast,
        East,
        Southeast,
        South,
        Southwest
    };

    const char* directionName(RelativeDirection direction) {
        switch (direction) {
            case RelativeDirection::West:
                return "West";
            case RelativeDirection::Northwest:
                return "Northwest";
            case RelativeDirection::North:
                return "North";
            case RelativeDirection::Northeast:
                return "Northeast";
            case RelativeDirection::East:
                return "East";
            case RelativeDirection::Southeast:
                return "Southeast";
            case RelativeDirection::South:
                return "South";
            case RelativeDirection::Southwest:
                return "Southwest";
        }

        return "Unknown";
    }

    bool directionFromChoice(
        int choice,
        RelativeDirection& direction
    ) {
        switch (choice) {
            case 1:
                direction = RelativeDirection::West;
                return true;
            case 2:
                direction = RelativeDirection::Northwest;
                return true;
            case 3:
                direction = RelativeDirection::North;
                return true;
            case 4:
                direction = RelativeDirection::Northeast;
                return true;
            case 5:
                direction = RelativeDirection::East;
                return true;
            case 6:
                direction = RelativeDirection::Southeast;
                return true;
            case 7:
                direction = RelativeDirection::South;
                return true;
            case 8:
                direction = RelativeDirection::Southwest;
                return true;
            default:
                return false;
        }
    }

    void printDirectionMenu() {
        std::cout << "Choose direction:\n"
                  << "1. West\n"
                  << "2. Northwest\n"
                  << "3. North\n"
                  << "4. Northeast\n"
                  << "5. East\n"
                  << "6. Southeast\n"
                  << "7. South\n"
                  << "8. Southwest\n";
    }

    // Convert a human-friendly direction and distance into a nearby GPS target.
    // The approximation is appropriate for the short movements allowed by the app.
    DroneAPI::LocationTarget moveTargetRelative(
        DroneAPI::LocationTarget target,
        RelativeDirection direction,
        double feet
    ) {
        double northFeet = 0.0;
        double eastFeet = 0.0;

        double diagonalFeet = feet / std::sqrt(2.0);

        switch (direction) {
            case RelativeDirection::West:
                eastFeet = -feet;
                break;
            case RelativeDirection::Northwest:
                northFeet = diagonalFeet;
                eastFeet = -diagonalFeet;
                break;
            case RelativeDirection::North:
                northFeet = feet;
                break;
            case RelativeDirection::Northeast:
                northFeet = diagonalFeet;
                eastFeet = diagonalFeet;
                break;
            case RelativeDirection::East:
                eastFeet = feet;
                break;
            case RelativeDirection::Southeast:
                northFeet = -diagonalFeet;
                eastFeet = diagonalFeet;
                break;
            case RelativeDirection::South:
                northFeet = -feet;
                break;
            case RelativeDirection::Southwest:
                northFeet = -diagonalFeet;
                eastFeet = -diagonalFeet;
                break;
        }

        double northMeters = northFeet * kFeetToMeters;
        double eastMeters = eastFeet * kFeetToMeters;

        double latitudeRadians = target.latitude * kPi / 180.0;

        target.latitude +=
            (northMeters / kEarthRadiusMeters) * 180.0 / kPi;

        target.longitude +=
            (eastMeters / (kEarthRadiusMeters * std::cos(latitudeRadians)))
            * 180.0 / kPi;

        return target;
    }

    // Use a great-circle distance so the base-radius check works directly with GPS.
    double horizontalDistanceMeters(
        double latitude1,
        double longitude1,
        double latitude2,
        double longitude2
    ) {
        double lat1Rad = latitude1 * kPi / 180.0;
        double lat2Rad = latitude2 * kPi / 180.0;

        double deltaLatRad = (latitude2 - latitude1) * kPi / 180.0;
        double deltaLonRad = (longitude2 - longitude1) * kPi / 180.0;

        double a =
            std::sin(deltaLatRad / 2.0) * std::sin(deltaLatRad / 2.0) +
            std::cos(lat1Rad) * std::cos(lat2Rad) *
            std::sin(deltaLonRad / 2.0) * std::sin(deltaLonRad / 2.0);

        double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));

        return kEarthRadiusMeters * c;
    }
}

DroneConsoleApp::DroneConsoleApp(DroneAPI& drone)
    : drone_(drone) {}

// The first choice determines whether this process should also launch PX4 SITL.
// After that, the same menu loop is used for simulated and external connections.
void DroneConsoleApp::run() {
    std::cout << "Run PX4 simulator? 1 = yes, 0 = no: ";

    int useSimulator = 0;
    std::cin >> useSimulator;

    usingSimulator_ = (useSimulator == 1);

    if (usingSimulator_) {
        setupSimulationHome();
    }

    int choice = -1;

    do {
        showMenu();
        choice = readInt("Enter a choice: ");

        switch (choice) {
            case 1:
                handleConnect();
                break;
            case 2:
                handleArm();
                break;
            case 3:
                handleDisarm();
                break;
            case 4:
                handleTakeoff();
                break;
            case 5:
                handleGoToLocation();
                break;
            case 6:
                handleToggleMovementLock();
                break;
            case 7:
                handleStatus();
                break;
            case 8:
                handleBattery();
                break;
            case 9:
                handleCurrentLocation();
                break;
            case 10:
                handleLand();
                break;
            case 11:
                handleReturnHome();
                break;
            case 12:
                handleMoveByVelocity();
                break;
            case 13:
                handleRelativeMission();
                break;
            case 14:
                handleGoToBaseLocation();
                break;
            case 0:
                std::cout << "Exiting drone console app.\n";
                break;
            default:
                std::cout << "Invalid choice.\n";
                break;
        }

        std::cout << '\n';

    } while (choice != 0);
}

void DroneConsoleApp::showMenu() const {
    std::cout << "            Drone Menu           \n"
              << "-------------------------------- \n"
              << "Movement lock: "
              << (movementCommandsLocked_ ? "LOCKED" : "UNLOCKED")
              << "\n"
              << "-------------------------------- \n"
              << " 1. Connect to Drone             \n"
              << " 2. Arm                          \n"
              << " 3. Disarm                       \n"
              << " 4. Takeoff                      \n"
              << " 5. Go to Location               \n"
              << " 6. Lock / Unlock Movement       \n"
              << " 7. Check Status                 \n"
              << " 8. Check Battery Life           \n"
              << " 9. Check Current Location       \n"
              << "10. Land                         \n"
              << "11. PX4 Return-to-Launch and Land\n"
              << "12. Move by Velocity             \n"
              << "13. Direction Step Mission       \n"
              << "14. Fly Back to Base Location    \n"
              << "    (No Landing)                 \n"
              << " 0. Exit                         \n";
}

void DroneConsoleApp::handleConnect() {
    std::cout << "Connect by:\n"
              << "1. Drone ID / simulation port\n"
              << "2. Custom MAVLink connection URL\n";

    int mode = readInt("Enter choice: ");

    if (mode == 1) {
        int droneID = readInt("Enter drone ID: ");

        if (drone_.connectToDrone(droneID)) {
            std::cout << "Connected to drone.\n";

            if (!usingSimulator_) {
                saveBaseLocationFromTelemetryIfNeeded();
            }
        } else {
            std::cout << "Failed to connect to drone.\n";
        }

        return;
    }

    if (mode == 2) {
        std::cout << "Example URLs:\n"
                  << "  udpin://0.0.0.0:14540\n"
                  << "  udpout://192.168.1.12:14550\n"
                  << "  serial:///dev/tty.usbserial-0001:57600\n";

        std::string connectionUrl;
        std::cout << "Enter MAVLink connection URL: ";
        std::cin >> connectionUrl;

        if (drone_.connectToDrone(connectionUrl)) {
            std::cout << "Connected to drone.\n";

            if (!usingSimulator_) {
                saveBaseLocationFromTelemetryIfNeeded();
            }
        } else {
            std::cout << "Failed to connect to drone.\n";
        }

        return;
    }

    std::cout << "Invalid connection mode.\n";
}

void DroneConsoleApp::handleArm() {
    if (drone_.arm()) {
        std::cout << "Drone armed.\n";
    } else {
        std::cout << "Failed to arm drone.\n";
    }
}

void DroneConsoleApp::handleDisarm() {
    if (drone_.disarm()) {
        std::cout << "Drone disarmed.\n";
    } else {
        std::cout << "Failed to disarm drone.\n";
    }
}

void DroneConsoleApp::handleTakeoff() {
    if (!requireMovementUnlocked("Takeoff")) {
        return;
    }

    if (!hasBaseLocation_) {
        saveBaseLocationFromTelemetryIfNeeded();
    }

    if (!hasBaseLocation_) {
        std::cout << "Cannot take off because BaseLocation is not available.\n";
        lockMovementCommands();
        return;
    }

    double altitudeMeters =
        readFeetAsMeters("Enter takeoff height in feet above home: ");

    if (drone_.takeoff(altitudeMeters)) {
        std::cout << "Takeoff completed.\n";
    } else {
        std::cout << "Takeoff failed.\n";
    }

    lockMovementCommands();
}

void DroneConsoleApp::handleGoToLocation() {
    if (!requireMovementUnlocked("Go to Location")) {
        return;
    }

    DroneAPI::LocationTarget target;

    target.latitude = readDouble("Enter target latitude: ");
    target.longitude = readDouble("Enter target longitude: ");
    target.relativeAltitudeMeters =
        readFeetAsMeters("Enter target height in feet above home: ");

    if (hasBaseLocation_) {
        double distanceFromBaseFeet =
            metersToFeet(
                horizontalDistanceMeters(
                    BaseLocation.latitude,
                    BaseLocation.longitude,
                    target.latitude,
                    target.longitude
                )
            );

        if (distanceFromBaseFeet > kMaxDistanceFromBaseFeet) {
            std::cout << std::fixed << std::setprecision(2)
                      << "That target is "
                      << distanceFromBaseFeet
                      << " feet from BaseLocation.\n"
                      << "Maximum allowed distance is "
                      << kMaxDistanceFromBaseFeet
                      << " feet.\n";

            lockMovementCommands();
            return;
        }
    }

    if (drone_.goToLocation(target)) {
        std::cout << "Go-to-location command completed.\n";
    } else {
        std::cout << "Go-to-location command failed.\n";
    }

    lockMovementCommands();
}

void DroneConsoleApp::handleGoToBaseLocation() {
    if (!requireMovementUnlocked("Fly Back to BaseLocation")) {
        return;
    }

    DroneAPI::DroneStatus status = drone_.status();

    if (!status.connected) {
        std::cout << "Connect to the drone first.\n";
        lockMovementCommands();
        return;
    }

    if (!hasBaseLocation_) {
        std::cout << "No BaseLocation is available.\n";
        lockMovementCommands();
        return;
    }

    if (!status.armed || !status.inAir) {
        std::cout << "Take off first before going to BaseLocation.\n";
        lockMovementCommands();
        return;
    }

    DroneAPI::CurrentLocation current = drone_.currentLocation();

    if (!std::isfinite(current.latitude) ||
        !std::isfinite(current.longitude) ||
        !std::isfinite(current.relativeAltitudeMeters)) {
        std::cout << "Current location is invalid. Cannot return to BaseLocation.\n";
        lockMovementCommands();
        return;
    }

    double distanceFeet =
        metersToFeet(
            horizontalDistanceMeters(
                current.latitude,
                current.longitude,
                BaseLocation.latitude,
                BaseLocation.longitude
            )
        );

    std::cout << std::fixed << std::setprecision(2)
              << "Distance to BaseLocation: "
              << distanceFeet
              << " feet.\n";

    DroneAPI::LocationTarget target{
        BaseLocation.latitude,
        BaseLocation.longitude,
        current.relativeAltitudeMeters
    };

    std::cout << "Going to BaseLocation at current altitude.\n";

    if (drone_.goToLocation(target)) {
        std::cout << "Reached BaseLocation.\n";
    } else {
        std::cout << "Failed to reach BaseLocation.\n";
    }

    lockMovementCommands();
}

void DroneConsoleApp::handleMoveByVelocity() {
    if (!requireMovementUnlocked("Move by Velocity")) {
        return;
    }

    DroneAPI::VelocityCommand command;

    std::cout << "Body-frame velocity command\n";
    std::cout << "Forward/right/down are in feet per second.\n";
    std::cout << "Yaw speed is in degrees per second.\n";

    command.northMetersPerSecond =
        feetToMeters(readDouble("Enter forward speed in ft/s: "));

    command.eastMetersPerSecond =
        feetToMeters(readDouble("Enter right speed in ft/s: "));

    command.downMetersPerSecond =
        feetToMeters(readDouble("Enter down speed in ft/s: "));

    command.yawDegreesPerSecond =
        readDouble("Enter yaw speed in deg/s: ");

    double durationSeconds =
        readDouble("Enter duration in seconds: ");

    if (drone_.moveByVelocity(command, durationSeconds)) {
        std::cout << "Velocity movement completed.\n";
    } else {
        std::cout << "Velocity movement failed.\n";
    }

    lockMovementCommands();
}

// Direction Step Mission is intentionally interactive. Each completed movement
// restores the lock and asks the user what should happen before another move.
void DroneConsoleApp::handleRelativeMission() {
    if (!requireMovementUnlocked("Direction Step Mission")) {
        return;
    }

    DroneAPI::DroneStatus status = drone_.status();

    if (!status.connected) {
        std::cout << "Connect to the drone first.\n";
        lockMovementCommands();
        return;
    }

    if (!status.armed || !status.inAir) {
        std::cout << "Take off first before running directional movement.\n";
        lockMovementCommands();
        return;
    }

    if (!hasBaseLocation_) {
        std::cout << "No BaseLocation is available. Cannot run directional movement.\n";
        lockMovementCommands();
        return;
    }

    bool keepGoing = true;

    while (keepGoing) {
        if (movementCommandsLocked_) {
            std::cout << "Movement commands are locked. Returning to main menu.\n";
            return;
        }

        DroneAPI::CurrentLocation current = drone_.currentLocation();

        if (!std::isfinite(current.latitude) ||
            !std::isfinite(current.longitude) ||
            !std::isfinite(current.relativeAltitudeMeters)) {
            std::cout << "Current location is invalid. Cannot build target.\n";
            lockMovementCommands();
            return;
        }

        printDirectionMenu();

        int directionChoice = readInt("Enter direction choice: ");

        RelativeDirection direction;

        if (!directionFromChoice(directionChoice, direction)) {
            std::cout << "Invalid direction choice.\n";
            continue;
        }

        double feet = readDouble("Enter distance in feet: ");

        if (!std::isfinite(feet) || feet <= 0.0) {
            std::cout << "Distance must be positive.\n";
            continue;
        }

        DroneAPI::LocationTarget target{
            current.latitude,
            current.longitude,
            current.relativeAltitudeMeters
        };

        target = moveTargetRelative(target, direction, feet);

        double distanceFromBaseFeet =
            metersToFeet(
                horizontalDistanceMeters(
                    BaseLocation.latitude,
                    BaseLocation.longitude,
                    target.latitude,
                    target.longitude
                )
            );

        if (distanceFromBaseFeet > kMaxDistanceFromBaseFeet) {
            std::cout << std::fixed << std::setprecision(2)
                      << "That move would place the drone "
                      << distanceFromBaseFeet
                      << " feet from BaseLocation.\n"
                      << "Maximum allowed distance is "
                      << kMaxDistanceFromBaseFeet
                      << " feet.\n";

            lockMovementCommands();
            return;
        }

        std::cout << "Command: go "
                  << directionName(direction)
                  << " for "
                  << feet
                  << " feet.\n";

        if (!drone_.goToLocation(target)) {
            std::cout << "Directional movement failed.\n";
        } else {
            std::cout << "Directional movement completed.\n";
        }

        lockMovementCommands();

        bool postMoveMenu = true;

        while (postMoveMenu) {
            std::cout << "What do you want to do next?\n"
                      << "1. Unlock and keep going\n"
                      << "2. Check live status\n"
                      << "3. Check battery life\n"
                      << "4. Check current location\n"
                      << "5. Unlock and fly back to BaseLocation\n"
                      << "6. Land\n"
                      << "0. Return to main menu\n";

            int nextChoice = readInt("Enter choice: ");

            if (nextChoice == 1) {
                movementCommandsLocked_ = false;
                std::cout << "Movement commands unlocked for next directional step.\n";
                postMoveMenu = false;
                keepGoing = true;
            } else if (nextChoice == 2) {
                handleStatus();
            } else if (nextChoice == 3) {
                handleBattery();
            } else if (nextChoice == 4) {
                handleCurrentLocation();
            } else if (nextChoice == 5) {
                movementCommandsLocked_ = false;
                std::cout << "Movement commands unlocked for return to BaseLocation.\n";

                handleGoToBaseLocation();

                postMoveMenu = false;
                keepGoing = false;
            } else if (nextChoice == 6) {
                postMoveMenu = false;
                keepGoing = false;

                std::cout << "Landing now.\n";

                if (drone_.land()) {
                    std::cout << "Landing completed.\n";
                } else {
                    std::cout << "Landing failed.\n";
                }
            } else if (nextChoice == 0) {
                postMoveMenu = false;
                keepGoing = false;
                std::cout << "Returning to main menu.\n";
            } else {
                std::cout << "Invalid choice.\n";
            }

            std::cout << '\n';
        }
    }
}

// The lock is a user-interface safeguard. Controller-side limits still apply even
// when this flag is unlocked.
void DroneConsoleApp::handleToggleMovementLock() {
    movementCommandsLocked_ = !movementCommandsLocked_;

    if (movementCommandsLocked_) {
        std::cout << "Movement commands are now LOCKED.\n";
    } else {
        std::cout << "Movement commands are now UNLOCKED.\n";
        std::cout << "Movement commands will automatically lock again after use.\n";
    }
}

bool DroneConsoleApp::requireMovementUnlocked(
    const std::string& commandName
) const {
    if (!movementCommandsLocked_) {
        return true;
    }

    std::cout << commandName
              << " is blocked because movement commands are locked.\n"
              << "Choose option 6 to unlock movement commands first.\n";

    return false;
}

void DroneConsoleApp::lockMovementCommands() {
    if (!movementCommandsLocked_) {
        movementCommandsLocked_ = true;
        std::cout << "Movement commands locked.\n";
    }
}

void DroneConsoleApp::handleStatus() const {
    DroneAPI::DroneStatus status = drone_.status();

    std::cout << "Connected: " << (status.connected ? "yes" : "no") << '\n'
              << "Armed: " << (status.armed ? "yes" : "no") << '\n'
              << "Healthy: " << (status.healthy ? "yes" : "no") << '\n'
              << "In air: " << (status.inAir ? "yes" : "no") << '\n';

    if (status.batteryPercent < 0) {
        std::cout << "Battery: unavailable\n";
    } else {
        std::cout << "Battery: " << status.batteryPercent << "%\n";
    }

    if (!status.flightMode.empty()) {
        std::cout << "Flight mode: " << status.flightMode << '\n';
    }

    if (!status.statusMessage.empty()) {
        std::cout << "Status message: " << status.statusMessage << '\n';
    }
}

void DroneConsoleApp::handleBattery() const {
    int battery = drone_.batteryLife();

    if (battery < 0) {
        std::cout << "Battery telemetry unavailable.\n";
    } else {
        std::cout << "Battery life: " << battery << "%\n";
    }
}

// Location output shows PX4 telemetry and, when possible, a separate online terrain
// estimate. The two sources may use different altitude references in simulation.
void DroneConsoleApp::handleCurrentLocation() const {
    DroneAPI::CurrentLocation location = drone_.currentLocation();

    std::cout << std::fixed << std::setprecision(6);

    std::cout << "Current location: "
              << location.latitude << ", "
              << location.longitude;

    std::cout << std::fixed << std::setprecision(2);

    std::cout << "\nRelative altitude: "
              << metersToFeet(location.relativeAltitudeMeters)
              << " feet above home"
              << "\nTelemetry altitude: "
              << metersToFeet(location.absoluteAltitudeMeters)
              << " feet AMSL reported by PX4\n";

    if (std::isfinite(location.latitude) &&
        std::isfinite(location.longitude) &&
        std::isfinite(location.absoluteAltitudeMeters)) {
        double terrainElevationMeters =
            ElevationService::elevationMeters(
                location.latitude,
                location.longitude
            );

        std::cout << std::fixed << std::setprecision(2)
                  << "Estimated real-world terrain elevation at current GPS: "
                  << metersToFeet(terrainElevationMeters)
                  << " feet above sea level\n";

        if (usingSimulator_) {
            std::cout << "Estimated height above local terrain: unavailable in simulation\n";
            std::cout << "Note: PX4 SITL altitude and real-world terrain elevation "
                      << "may use different references.\n";
        } else {
            double estimatedHeightAboveTerrainFeet =
                metersToFeet(
                    location.absoluteAltitudeMeters -
                    terrainElevationMeters
                );

            std::cout << "Estimated height above local terrain: "
                      << estimatedHeightAboveTerrainFeet
                      << " feet\n";

            std::cout << "Note: This is an estimate based on PX4 telemetry altitude "
                      << "minus online terrain elevation. Do not use it as the only "
                      << "source for safety-critical altitude decisions.\n";
        }
    } else {
        std::cout << "Estimated real-world terrain elevation at current GPS: unavailable\n";
    }

    printDistanceToBaseLocation();
}

void DroneConsoleApp::handleLand() {
    if (drone_.land()) {
        std::cout << "Landing completed.\n";
    } else {
        std::cout << "Landing failed.\n";
    }
}

void DroneConsoleApp::handleReturnHome() {
    if (!requireMovementUnlocked("PX4 Return-to-Launch")) {
        return;
    }

    if (drone_.returnToHome()) {
        std::cout << "Return home command completed.\n";
    } else {
        std::cout << "Return home failed.\n";
    }

    lockMovementCommands();
}

void DroneConsoleApp::saveBaseLocationFromTelemetryIfNeeded() {
    if (hasBaseLocation_) {
        return;
    }

    DroneAPI::CurrentLocation current = drone_.currentLocation();

    if (!std::isfinite(current.latitude) ||
        !std::isfinite(current.longitude)) {
        std::cout << "BaseLocation could not be saved yet. "
                  << "Current GPS telemetry is unavailable.\n";
        return;
    }

    BaseLocation = DroneAPI::LocationTarget{
        current.latitude,
        current.longitude,
        0.0
    };

    hasBaseLocation_ = true;

    std::cout << std::fixed << std::setprecision(6)
              << "BaseLocation saved from current drone telemetry: "
              << BaseLocation.latitude
              << ", "
              << BaseLocation.longitude
              << '\n';
}

int DroneConsoleApp::readInt(const std::string& prompt) const {
    int value;

    while (true) {
        std::cout << prompt;

        if (std::cin >> value) {
            return value;
        }

        std::cout << "Invalid number. Try again.\n";

        std::cin.clear();
        std::cin.ignore(
            std::numeric_limits<std::streamsize>::max(),
            '\n'
        );
    }
}

double DroneConsoleApp::readDouble(const std::string& prompt) const {
    double value;

    while (true) {
        std::cout << prompt;

        if (std::cin >> value) {
            return value;
        }

        std::cout << "Invalid number. Try again.\n";

        std::cin.clear();
        std::cin.ignore(
            std::numeric_limits<std::streamsize>::max(),
            '\n'
        );
    }
}

void DroneConsoleApp::printDistanceToBaseLocation() const {
    if (!hasBaseLocation_) {
        std::cout << "Distance to BaseLocation: unavailable\n";
        return;
    }

    DroneAPI::CurrentLocation current = drone_.currentLocation();

    if (!std::isfinite(current.latitude) ||
        !std::isfinite(current.longitude)) {
        std::cout << "Distance to BaseLocation: unavailable\n";
        return;
    }

    double distanceFeet =
        metersToFeet(
            horizontalDistanceMeters(
                current.latitude,
                current.longitude,
                BaseLocation.latitude,
                BaseLocation.longitude
            )
        );

    std::cout << std::fixed << std::setprecision(2)
              << "Distance to BaseLocation: "
              << distanceFeet
              << " feet\n";
}

// A simulator home point is saved as the base before PX4 starts, so distance checks
// are available as soon as the vehicle connects.
void DroneConsoleApp::setupSimulationHome() {
    std::cout << "Set simulator starting GPS location\n";
    std::cout << "-----------------------------------\n";

    homeLocation_.latitude = readDouble("Enter home latitude: ");
    homeLocation_.longitude = readDouble("Enter home longitude: ");

    homeLocation_.altitudeMeters =
        ElevationService::elevationMeters(
            homeLocation_.latitude,
            homeLocation_.longitude
        );

    BaseLocation = DroneAPI::LocationTarget{
        homeLocation_.latitude,
        homeLocation_.longitude,
        0.0
    };

    hasBaseLocation_ = true;

    std::cout << std::fixed << std::setprecision(6)
              << "BaseLocation saved: "
              << BaseLocation.latitude
              << ", "
              << BaseLocation.longitude
              << '\n';

    std::cout << std::fixed << std::setprecision(2)
              << "Estimated home altitude: "
              << metersToFeet(homeLocation_.altitudeMeters)
              << " feet above sea level\n";

    PX4SimulatorLauncher launcher("../PX4-Autopilot");

    launcher.launch(homeLocation_);
}

double DroneConsoleApp::readFeetAsMeters(const std::string& prompt) const {
    double feet = readDouble(prompt);
    return feetToMeters(feet);
}

double DroneConsoleApp::feetToMeters(double feet) {
    return feet * 0.3048;
}

double DroneConsoleApp::metersToFeet(double meters) {
    return meters / 0.3048;
}
