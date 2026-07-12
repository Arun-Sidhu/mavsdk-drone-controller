# Drone Controller

This is a personal C++ project for experimenting with drone control through a shared API. It includes a mock controller for testing the basic command flow and a MAVSDK controller that can communicate with PX4 Software-in-the-Loop simulation or a compatible vehicle.

The project grew out of my interest in connecting higher-level application code to drone flight commands while keeping the controller behind a clear interface. The console application supports connection management, arming, takeoff, location-based movement, body-frame velocity commands, telemetry, landing, and return-to-launch behavior.

## Project structure

```text
mavsdk-drone-controller/
├── app/
│   ├── DroneConsoleApp.cpp
│   ├── DroneConsoleApp.h
│   ├── ElevationService.cpp
│   ├── ElevationService.h
│   ├── PX4SimulatorLauncher.cpp
│   └── PX4SimulatorLauncher.h
├── mock/
│   ├── main.cpp
│   ├── MockDroneController.cpp
│   └── MockDroneController.h
├── real/
│   ├── main.cpp
│   ├── DroneCommandUtils.cpp
│   ├── DroneCommandUtils.h
│   ├── DroneSafetyLimits.h
│   ├── MavsdkDroneController.cpp
│   ├── MavsdkDroneController.h
│   ├── MavsdkTelemetryUtils.cpp
│   └── MavsdkTelemetryUtils.h
├── .gitignore
├── DroneAPI.h
├── Makefile
└── README.md
```

`DroneAPI.h` defines the common interface used by both controllers. The mock controller keeps its own in-memory state, which makes it useful for checking the API without starting PX4. The MAVSDK controller handles MAVLink communication and includes validation and safety checks before sending flight commands.

## Requirements

The mock build only needs a C++ compiler and `make`.

The MAVSDK build also requires:

- MAVSDK C++
- `pkg-config`
- `curl`
- PX4-Autopilot and its simulator dependencies

The Makefile expects the following command to return the MAVSDK compiler and linker flags:

```bash
pkg-config --cflags --libs mavsdk
```

MAVSDK installation instructions are available in the [official MAVSDK guide](https://mavsdk.mavlink.io/main/en/cpp/guide/installation.html).

## Getting PX4-Autopilot

PX4-Autopilot is maintained as a separate project and is not included in this repository. It should be cloned beside this repository so the simulator launcher can find it through the relative path used by the application.

From the parent folder containing `mavsdk-drone-controller`, run:

```bash
git clone --recursive https://github.com/PX4/PX4-Autopilot.git
```

The folders should be arranged like this:

```text
Drones_Project/
├── mavsdk-drone-controller/
└── PX4-Autopilot/
```

The application expects this layout because the simulator launcher uses:

```cpp
PX4SimulatorLauncher launcher("../PX4-Autopilot");
```

Before using the simulator, follow the [PX4 development setup instructions](https://docs.px4.io/main/en/dev_setup/) to install the required tools and dependencies.

You can confirm that PX4 builds and starts correctly by running:

```bash
cd PX4-Autopilot
make px4_sitl gz_x500
```

Once the simulator opens successfully, stop it and return to the `mavsdk-drone-controller` folder.

## Building and running the mock controller

From the repository root, build and run the mock controller with:

```bash
make mock
./mock_drone
```

The same steps can be run with one command:

```bash
make run-mock
```

Running `make` by itself also builds the mock controller because it is the default target.

The mock program connects to an in-memory drone, arms it, takes off, moves to an example location, prints telemetry, and lands. It does not require PX4 or MAVSDK.

## Building and running the MAVSDK controller

Build the MAVSDK version with:

```bash
make real
```

Run it with:

```bash
make run-real
```

To stop old PX4, Gazebo, or controller processes before starting a new session, use:

```bash
make run-real-fresh
```

To perform the same cleanup before launch and again after the application exits, use:

```bash
make run-real-clean
```

The application asks whether PX4 SITL should be started. When simulation is selected, it asks for a home latitude and longitude, looks up the terrain elevation, starts the PX4 x500 simulation, and waits briefly for it to finish booting.

## Console safety behavior

Movement commands begin in a locked state. They must be unlocked before takeoff, location movement, velocity movement, or return-to-launch can be sent. The application locks them again after a movement command finishes.

The project also checks values such as altitude, horizontal distance, velocity, yaw speed, and command duration. These checks are useful safeguards during development, but this project should not be treated as a complete flight-safety system.

## Useful Makefile commands

```bash
make                 # Build the mock controller
make mock            # Build the mock controller
make real            # Build the MAVSDK controller
make run-mock        # Build and run the mock controller
make run-real        # Build and run the MAVSDK controller
make check-sim       # Show related running processes
make stop-sim        # Stop related simulator and controller processes
make run-real-fresh  # Clean up old processes, then build and run
make run-real-clean  # Clean up before and after the run
make clean           # Remove the compiled executables
make clean-all       # Stop related processes and remove executables
```

## Elevation lookup

`ElevationService` uses `curl` to request terrain elevation from the Open-Meteo elevation API. If the request fails or the response cannot be parsed, the application falls back to an elevation of zero and prints a warning.

The terrain value is only an estimate. It should not be used as the only source for safety-critical altitude decisions.

## Current status

This is an ongoing personal project. The mock controller is useful for testing the shared API, while the MAVSDK controller and console application provide a foundation for experimenting with PX4 simulation, telemetry, flight commands, and additional safety rules.
