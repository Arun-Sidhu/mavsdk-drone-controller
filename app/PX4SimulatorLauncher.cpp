/*
 * This implementation starts PX4 SITL as a background process, writes its
 * output to /tmp/px4_sitl.log, and gives the simulator a short period to boot.
 * It keeps the launch process straightforward and is intended for the macOS
 * development setup used by this project.
 */

#include "PX4SimulatorLauncher.h"

#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>
#include <chrono>

PX4SimulatorLauncher::PX4SimulatorLauncher(std::string px4Path)
    : px4Path_(std::move(px4Path)) {}

// The command is asynchronous so the console can continue and connect after the
// short startup wait. PX4 output remains available in the log file for debugging.
bool PX4SimulatorLauncher::launch(
    const PX4SimulatorLauncher::HomeLocation& homeLocation
) const {
    std::ostringstream command;

	command << std::fixed << std::setprecision(8);
	
	command << "cd " << px4Path_ << " && "
			<< "PX4_HOME_LAT=" << homeLocation.latitude << " "
			<< "PX4_HOME_LON=" << homeLocation.longitude << " "
			<< std::setprecision(3)
			<< "PX4_HOME_ALT=" << homeLocation.altitudeMeters << " "
			<< "make px4_sitl gz_x500 "
			<< "> /tmp/px4_sitl.log 2>&1 &";

    std::cout << "Starting PX4 SITL at:\n"
			  << std::fixed << std::setprecision(6)
			  << "Latitude: " << homeLocation.latitude << '\n'
			  << "Longitude: " << homeLocation.longitude << '\n'
			  << std::setprecision(2)
			  << "Altitude: "
			  << homeLocation.altitudeMeters / 0.3048
			  << " feet\n";

    int result = std::system(command.str().c_str());

    if (result != 0) {
        std::cerr << "Failed to start PX4 SITL.\n";
        return false;
    }

    std::cout << "PX4 SITL launch command sent.\n";
    std::cout << "Waiting for simulator to start...\n";

    std::this_thread::sleep_for(std::chrono::seconds(20));

    return true;
}
