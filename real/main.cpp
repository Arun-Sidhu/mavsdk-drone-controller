/*
 * This file is the composition point for the interactive application. It creates
 * the MAVSDK-backed controller, passes it to the console, and then lets the
 * console own the user-facing program flow.
 */

// real
#include "MavsdkDroneController.h"
#include "../app/DroneConsoleApp.h"

int main() {
    MavsdkDroneController drone;
    DroneConsoleApp app(drone);

    app.run();

    return 0;
}
