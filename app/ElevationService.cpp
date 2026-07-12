/*
 * The elevation lookup is intentionally lightweight. It calls the Open-Meteo
 * elevation endpoint through curl and extracts the single value needed by the
 * application. A failed lookup is reported and falls back to zero meters, which
 * is useful for keeping a simulator session moving but is not a reliable value
 * for real-world flight decisions.
 */

#include "ElevationService.h"

#include <array>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <regex>
#include <sstream>
#include <string>

// curl is used here to keep the project free of another C++ HTTP dependency.
double ElevationService::elevationMeters(double latitude, double longitude) {
    std::ostringstream url;

    url << std::fixed << std::setprecision(8)
        << "https://api.open-meteo.com/v1/elevation"
        << "?latitude=" << latitude
        << "&longitude=" << longitude;

    std::string command = "curl -fsSL \"" + url.str() + "\"";

    std::array<char, 256> buffer{};
    std::string response;

    FILE* pipe = popen(command.c_str(), "r");

    if (!pipe) {
        std::cerr << "Could not run curl. Using 0 feet for elevation.\n";
        return 0.0;
    }

    while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
        response += buffer.data();
    }

    int status = pclose(pipe);

    if (status != 0) {
        std::cerr << "Elevation lookup failed. Using 0 feet for elevation.\n";
        return 0.0;
    }

    // The endpoint returns arrays even for a single coordinate, so the expression
    // extracts the first numeric value from the elevation array.
    std::regex elevationPattern(
        R"("elevation"\s*:\s*\[\s*(-?\d+(?:\.\d+)?)\s*\])"
    );

    std::smatch match;

    if (!std::regex_search(response, match, elevationPattern)) {
        std::cerr << "Could not parse elevation response: "
                  << response
                  << "\nUsing 0 feet for elevation.\n";
        return 0.0;
    }

    double elevationMeters = std::stod(match[1].str());
    
    std::cout << "Looked up terrain elevation: "
    		  << elevationMeters / 0.3048
    		  << " feet\n";
    		  
    return elevationMeters;
}
