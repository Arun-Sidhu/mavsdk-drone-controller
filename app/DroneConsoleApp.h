/*
 * DroneConsoleApp declares the interactive command-line interface for the
 * project. The class does not own a controller. Instead, it receives any
 * DroneAPI implementation and turns menu choices into calls on that interface.
 * It also keeps track of the base location and the one-command movement lock.
 */

#pragma once

#include "../DroneAPI.h"
#include "PX4SimulatorLauncher.h"

#include <string>

// The console keeps presentation and input handling separate from flight control.
// That separation is what allows the same menu to use different controllers.
class DroneConsoleApp {
public:
    explicit DroneConsoleApp(DroneAPI& drone);

    void run();

private:
    DroneAPI& drone_;

    PX4SimulatorLauncher::HomeLocation homeLocation_;

    DroneAPI::LocationTarget BaseLocation{};
    bool hasBaseLocation_ = false;
    bool usingSimulator_ = false;
    // A movement command must be deliberately unlocked and the lock is restored
    // after use. This makes an accidental repeated menu selection less likely.
    bool movementCommandsLocked_ = true;

    void setupSimulationHome();

    void showMenu() const;

    void handleConnect();
    void handleArm();
    void handleDisarm();
    void handleTakeoff();
    void handleGoToLocation();
    void handleRelativeMission();
    void handleMoveByVelocity();
    void handleStatus() const;
    void handleBattery() const;
    void handleCurrentLocation() const;
    void handleLand();
    void handleReturnHome();

    void handleGoToBaseLocation();
    void handleToggleMovementLock();

    bool requireMovementUnlocked(const std::string& commandName) const;
    void lockMovementCommands();

    void saveBaseLocationFromTelemetryIfNeeded();
    void printDistanceToBaseLocation() const;

    int readInt(const std::string& prompt) const;
    double readDouble(const std::string& prompt) const;
    double readFeetAsMeters(const std::string& prompt) const;

    static double feetToMeters(double feet);
    static double metersToFeet(double meters);
};
