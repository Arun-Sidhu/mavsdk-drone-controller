/*
 * PX4SimulatorLauncher keeps the shell command used to start PX4 SITL out of
 * the console application. The launcher receives a GPS home point, passes it
 * to PX4 through environment variables, and starts the Gazebo x500 model.
 */

#pragma once

#include <string>

class PX4SimulatorLauncher {
public:
	// Volunteer Park Water Tower
    // These defaults are only a fallback. The console normally asks for a home
    // latitude and longitude before launching the simulator.
    struct HomeLocation {
        double latitude = 47.628990;
        double longitude = -122.314406;
        double altitudeMeters = 520;
    };

    explicit PX4SimulatorLauncher(std::string px4Path);

    bool launch(const HomeLocation& homeLocation) const;

private:
    std::string px4Path_;
};
