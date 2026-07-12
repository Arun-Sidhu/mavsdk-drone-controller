# This Makefile keeps the lightweight mock build separate from the MAVSDK build.
# Running plain `make` builds the mock version, which is useful for checking the
# shared interface without needing PX4, Gazebo, or MAVSDK installed.

CXX := clang++

MOCK_TARGET := mock_drone
REAL_TARGET := real_drone

# The mock executable only needs its small in-memory controller and test entry point.
MOCK_SOURCES := mock/main.cpp mock/MockDroneController.cpp

# The real executable includes the MAVSDK controller and the shared console application.
REAL_SOURCES := real/main.cpp real/MavsdkDroneController.cpp real/DroneCommandUtils.cpp real/MavsdkTelemetryUtils.cpp app/DroneConsoleApp.cpp app/PX4SimulatorLauncher.cpp app/ElevationService.cpp

# The mock currently compiles with C++17. The real controller uses C++20 features,
# including std::numbers in the coordinate utility code.
MOCK_FLAGS := -std=c++17 -Wall -Wextra -pedantic -I.
REAL_FLAGS := -std=c++20 -Wall -Wextra -I.

# pkg-config supplies the include and linker settings for the local MAVSDK install.
# Redirecting errors keeps mock-only workflows quiet on systems without MAVSDK.
MAVSDK_CFLAGS := $(shell pkg-config --cflags mavsdk 2>/dev/null)
MAVSDK_LIBS := $(shell pkg-config --libs mavsdk 2>/dev/null)

.PHONY: all mock real run-mock run-real clean check-sim stop-sim run-real-fresh run-real-clean clean-all

# The mock is the default because it has the fewest external requirements.
all: mock

mock:
	$(CXX) $(MOCK_FLAGS) $(MOCK_SOURCES) -o $(MOCK_TARGET)

real:
	$(CXX) $(REAL_FLAGS) $(MAVSDK_CFLAGS) $(REAL_SOURCES) -o $(REAL_TARGET) $(MAVSDK_LIBS)

run-mock: mock
	./$(MOCK_TARGET)

run-real: real
	./$(REAL_TARGET)

clean:
	rm -f $(MOCK_TARGET) $(REAL_TARGET)

# This is a quick way to see whether an older PX4, Gazebo, or console process
# is still running before starting another simulation session.
check-sim:
	@ps aux | grep -E "px4|gz sim|real_drone" | grep -v grep || true

# Old simulator processes can keep ports or Gazebo resources busy. This target
# stops the processes used by this project and continues even when none are found.
stop-sim:
	@echo "Stopping old PX4, Gazebo, and real_drone processes..."
	@pkill -f "PX4-Autopilot/build/px4_sitl_default/bin/px4" || true
	@pkill -f "PX4_SIM_MODEL=gz_x500" || true
	@pkill -f "gz sim" || true
	@pkill -f "real_drone" || true
	@sleep 2
	@echo "Simulator cleanup complete."

# Start from a clean simulator state, rebuild the real application, and run it.
run-real-fresh: stop-sim real
	./$(REAL_TARGET)

# This version also stops the simulator processes after the console exits.
run-real-clean: stop-sim real
	./$(REAL_TARGET); $(MAKE) stop-sim

clean-all: stop-sim clean
