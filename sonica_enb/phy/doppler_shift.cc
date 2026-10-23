/*
 * Copyright 2025-2026 LAAS-CNRS
 *
 * This file is part of Sonica.
 *
 * Sonica is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * Sonica is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

#include <iostream>
#include <math.h>
#include <fstream>
#include <ctime>
#include <chrono>
#include <tuple>
#include <string>
#include <vector>
#include "sonica_enb/hdr/phy/doppler_shift.h"

// ----- Parameters ----- //
const double H = 600000;  // Height of satellite (m)
const double Theta_min = 30;  // Minimum elevation angle (degree)
const double f = 2154.8975e6;  // Carrier frequency (Hz)
// ----- Constants ----- //
const double K = 3.986005 * pow(10, 14);  // Kepler constant of Earth
const double R_e = 6371000;  // Radius of Earth (m)
const double R = R_e + H;  // Orbit radius (m)
const double c = 299792458;  // Speed of light (m/s)
const double Omega_s = sqrt(K / pow(H + R_e, 3));  // Angular velocity of the satellite in the ECF frame (s^-1)



namespace sonica_enb {
    // Experiment time from the local clock, t = 0 at maximum elevation
    double doppler_shift::get_experiment_time(double visibility_time, double start_time) {
        auto time = std::chrono::system_clock::now().time_since_epoch();
        std::chrono::seconds seconds = std::chrono::duration_cast< std::chrono::seconds >(time);
        std::chrono::milliseconds ms = std::chrono::duration_cast< std::chrono::milliseconds >(time);
        double now = (double) seconds.count() + ((double) (ms.count() % 1000)/1000.0);
        //time(&now);
        double elapsed_time = now - start_time;
        double experiment_time = (- visibility_time / 2) + elapsed_time;
        return experiment_time;
    }

    // Visibility time as a function of maximum elevation angle
    double doppler_shift::t_v(double x)
    {
        double p = cos(acos(R_e / R * cos(Theta_min * M_PI / 180.0)) - Theta_min * M_PI / 180.0);
        return (2 / Omega_s) * acos(p / cos(acos(R_e / R * cos(x * M_PI / 180.0)) - x * M_PI / 180.0));
    };

    // Compute Doppler effect for a given elevation angle and time
    double doppler_shift::get_doppler_effect(double Theta_max, double t) {
        double a1 = cos(acos(R_e / R * cos(Theta_max * M_PI / 180.0)) - Theta_max * M_PI / 180.0);
        double delta_f = -f / c * (R_e * R * sin(Omega_s * t)) * a1 * Omega_s / sqrt(pow(R_e, 2) + pow(R, 2) - 2 * R_e * R * cos(Omega_s * t) * a1);
        return delta_f;
    }



    std::tuple<int, double, float> doppler_shift::get_parameters() {
        std::ifstream file("../../../config.txt"); // Open the file
        std::string line;
        int maxElevation = 90; // Default value if not found
        double carrierFrequency = 2.4e9;
        float awgn = 0.0f; // Default value for AWGN

        if (!file) {
            std::cerr << "Error: Unable to open file!" << std::endl;
            return {0, 0.0, 0.0f}; // Return default values
        }

        while (std::getline(file, line)) { // Read file line by line
            if (line.find("Maximum elevation = ") != std::string::npos) {
                size_t pos = line.find("Maximum elevation =");
                if (pos != std::string::npos) {
                    std::string value = line.substr(pos + 20); // Extract value after '='
                    maxElevation = std::stoi(value); // Convert to int using std::stoi
                }
            }
            else if (line.find("Carrier frequency = ") != std::string::npos) {
                size_t pos = line.find("Carrier frequency =");
                if (pos != std::string::npos) {
                    std::string value = line.substr(pos + 19); // Extract value after '='
                    carrierFrequency = std::stod(value); // Convert to double using std::stod
                }
            }
            else if (line.find("AWGN = ") != std::string::npos) {
                size_t pos = line.find("AWGN =");
                if (pos != std::string::npos) {
                    std::string value = line.substr(pos + 7); // Extract value after '='
                    awgn = std::stof(value); // Convert to float using std::stof
                }
            }
        }

        file.close();

        // Output the extracted values
        if (maxElevation != -1 && carrierFrequency != -1.0 && awgn != -1.0f) {
            std::cout << "Maximum elevation: " << maxElevation << std::endl;
            std::cout << "Carrier frequency: " << carrierFrequency << std::endl;
            std::cout << "AWGN: " << awgn << std::endl;
        } else {
            std::cout << "Required values not found!" << std::endl;
            return {0, 0.0, 0.0f}; // Return default values
        }

        return {maxElevation, carrierFrequency, awgn}; // Return all three values as a tuple
    }

    double doppler_shift::get_fspl() {
        std::ifstream file("../../../config.txt");
        std::string line;
        double fspl = 0.0;

        if (!file) {
            std::cerr << "Error: Unable to open file!" << std::endl;
            return 0.0;
        }

        while (std::getline(file, line)) {
            if (line.find("FSPL = ") != std::string::npos) {
                size_t pos = line.find("FSPL = ");
                if (pos != std::string::npos) {
                    std::string value = line.substr(pos + 7); // Skip "FSPL = "
                    try {
                        fspl = std::stod(value);
                    } catch (const std::exception& e) {
                        std::cerr << "Error parsing FSPL: " << e.what() << std::endl;
                        return 0.0;
                    }
                    break; // Stop reading after FSPL is found
                }
            }
        }

        file.close();

        std::cout << "FSPL: " << fspl << std::endl;
        return fspl;
    }

    std::string doppler_shift::get_remote_ip() {
        std::ifstream file("../../../config.txt");
        std::string line;
        std::string remote_ip;

        if (!file) {
            std::cerr << "Error: Unable to open file!" << std::endl;
            return "";
        }

        while (std::getline(file, line)) {
            if (line.find("REMOTE_IP = ") != std::string::npos) {
                size_t pos = line.find("REMOTE_IP = ");
                if (pos != std::string::npos) {
                    remote_ip = line.substr(pos + 12); // Skip "REMOTE_IP = "
                    remote_ip.erase(remote_ip.find_last_not_of(" \t\r\n") + 1);
                    break; // Stop reading after REMOTE_IP is found
                }
            }
        }

        file.close();

        if (remote_ip.empty()) {
            std::cerr << "Error: REMOTE_IP not found in config!" << std::endl;
        } else {
            std::cout << "Remote IP: " << remote_ip << std::endl;
        }

        return remote_ip;
    }

    std::vector<double> doppler_shift::precompute_doppler(double Theta_max, double t_start, double t_end, double dt) {
        std::vector<double> doppler_values;
        size_t num_points = static_cast<size_t>((t_end - t_start) / dt) + 1;
        doppler_values.reserve(num_points);

        for (size_t i = 0; i < num_points; ++i) {
            double t = t_start + i * dt;
            double a1 = cos(acos(R_e / R * cos(Theta_max * M_PI / 180.0)) - Theta_max * M_PI / 180.0);
            double delta_f = -f / c * (R_e * R * sin(Omega_s * t)) * a1 * Omega_s /
                            sqrt(pow(R_e, 2) + pow(R, 2) - 2 * R_e * R * cos(Omega_s * t) * a1);
            doppler_values.push_back(delta_f);
        }

        return doppler_values;
    }

    double doppler_shift::get_precomputed_doppler(const std::vector<double>& doppler_values,
                               double t_start, double dt, double t) {
    size_t index = static_cast<size_t>((t - t_start) / dt);
    if (index >= doppler_values.size()) index = doppler_values.size() - 1;  // clamp to last element
    return doppler_values[index];
}
}
