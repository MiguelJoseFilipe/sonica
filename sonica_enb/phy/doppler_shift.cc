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
#include <algorithm>
#include <cstdio>
#include <cstdlib>
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

bool qef_model = false;
std::vector<double> qef_amp_table;
double qef_t0 = 0.0;
double qef_dt = 0.1;
std::vector<double> qef_sat_table[6];



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

    // ----- Quasi-earth-fixed model, as in the OAI gNB (nr_doppler_shift.c) ----- //
    // The satellite pre-compensates the beam-center Doppler, so the UE sees the residual
    //   -f_c/c * (rr_device - rr_center).
    // Beam gain ("Beam radius km" > 0): DL amplitude a(t) = 10^(offset/20) * g(theta) * H / range_device,
    // g = TR 38.811 6.4.1 aperture pattern (amplitude 2|J1(x)/x|, x = ka sin theta), ka from the HPBW
    // of a nadir beam whose -3 dB edge lies at the given ground radius, theta = off-axis angle.

    static const double X3DB = 1.6163399483107;  // 2 J1(x)/x = 1/sqrt(2): the -3 dB point

    // Half HPBW (rad) of a nadir beam whose -3 dB edge is r_m (m) from the sub-satellite point
    static double beam_half_hpbw(double r_m) {
        const double g = r_m / R_e;  // central angle
        return atan2(R_e * sin(g), H + R_e * (1.0 - cos(g)));
    }

    // Table CSV ('#' header, rows evenly spaced in time), from make_qef_table.py:
    //   t_s, rr_center, rr_device, range_device, sat_x, sat_y, sat_z, sat_vx, sat_vy, sat_vz,
    //   masked[, off_axis_deg]
    // Returns false when the config does not select the QEF model; exits on a bad table, so a
    // QEF experiment never runs silently without its Doppler.
    bool doppler_shift::load_qef(double carrier_hz) {
        const std::string cfg_path = "../../../config.txt";
        std::ifstream cfg(cfg_path);
        int model = 0, window = 1;
        std::string table = "qef_table.csv";
        double beam_km = 0.0, offset_db = 0.0;
        std::string line;
        char buf[256];
        while (std::getline(cfg, line)) {
            if (line.empty() || line[0] == '#') continue;  // comments may mention key names
            const char* l = line.c_str();
            if (sscanf(l, "Doppler model = %d", &model) == 1) {}
            else if (sscanf(l, "QEF window = %d", &window) == 1) {}
            else if (sscanf(l, "QEF table = %255s", buf) == 1) table = buf;
            else if (sscanf(l, "Beam radius km = %lf", &beam_km) == 1) {}
            else sscanf(l, "Power offset dB = %lf", &offset_db);
        }
        if (model != 1) {
            if (model != 0) std::cerr << "[DOPPLER] invalid Doppler model " << model << ", using earth-moving" << std::endl;
            return false;
        }
        if (beam_km < 0.0 || beam_km > 2000.0) {
            std::cerr << "[DOPPLER] invalid Beam radius km " << beam_km << ", beam gain off" << std::endl;
            beam_km = 0.0;
        }
        if (offset_db > 0.0)
            std::cerr << "[DOPPLER] Power offset dB " << offset_db << " > 0: TX samples may clip" << std::endl;

        // A relative table path is resolved from the directory of config.txt
        const std::string path = table[0] == '/' ? table
                               : cfg_path.substr(0, cfg_path.rfind('/') + 1) + table;
        std::ifstream f(path);
        std::vector<double> t, off;
        std::vector<int> masked;
        std::vector<double> rr_c, rr_d, range, sat[6];
        while (std::getline(f, line)) {
            if (line.empty() || line[0] == '#') continue;
            double ts, rc, rd, rg, p[6], od = NAN;
            int m;
            if (sscanf(line.c_str(), "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%lf",
                       &ts, &rc, &rd, &rg, &p[0], &p[1], &p[2], &p[3], &p[4], &p[5], &m, &od) >= 11) {
                t.push_back(ts); rr_c.push_back(rc); rr_d.push_back(rd); range.push_back(rg);
                masked.push_back(m); off.push_back(od * M_PI / 180.0);
                for (int k = 0; k < 6; k++) sat[k].push_back(p[k]);
            }
        }
        const size_t n = t.size();
        if (n < 2) {
            std::cerr << "[DOPPLER] QEF table " << path << ": unreadable or fewer than 2 rows" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        // Window: the whole pass, or the rows where center and device clear the elevation mask
        size_t first = 0, last = n - 1;
        if (window) {
            while (first < n && !masked[first]) first++;
            while (last > first && !masked[last]) last--;
            if (first >= n || !masked[last]) {
                std::cerr << "[DOPPLER] QEF table " << path << ": no masked rows" << std::endl;
                std::exit(EXIT_FAILURE);
            }
        }
        const double dt = t[first + 1] - t[first];
        for (size_t i = first; i <= last; i++) {
            if (fabs(t[i] - t[first] - (double)(i - first) * dt) > 1e-3) {
                std::cerr << "[DOPPLER] QEF table " << path << ": rows are not evenly spaced (row " << i << ")" << std::endl;
                std::exit(EXIT_FAILURE);
            }
        }

        doppler_table.clear();
        qef_amp_table.clear();
        for (int k = 0; k < 6; k++)
            qef_sat_table[k].assign(sat[k].begin() + first, sat[k].begin() + last + 1);
        const double ka = X3DB / sin(beam_half_hpbw(beam_km * 1000.0));
        const double k0 = pow(10.0, offset_db / 20.0);
        for (size_t i = first; i <= last; i++) {
            doppler_table.push_back(-(carrier_hz / c) * (rr_d[i] - rr_c[i]));
            if (beam_km > 0.0) {
                if (std::isnan(off[i])) {
                    std::cerr << "[DOPPLER] QEF table " << path << " has no off_axis_deg column: regenerate it for the beam gain" << std::endl;
                    std::exit(EXIT_FAILURE);
                }
                const double x = ka * sin(off[i]);
                const double g = fabs(x) < 1e-9 ? 1.0 : fabs(2.0 * j1(x) / x);
                qef_amp_table.push_back(k0 * g * H / range[i]);
            }
        }
        qef_t0 = t[first];
        qef_dt = dt;

        const auto dop = std::minmax_element(doppler_table.begin(), doppler_table.end());
        printf("[DOPPLER] QEF table %s: %zu of %zu rows, t=[%.2f, %.2f] s, dt=%.3f s, %s window\n",
               path.c_str(), last - first + 1, n, t[first], t[last], dt, window ? "elevation-masked" : "full-pass");
        printf("[DOPPLER] QEF residual Doppler %.1f .. %.1f Hz at %.7e Hz\n", *dop.first, *dop.second, carrier_hz);
        if (!qef_amp_table.empty()) {
            const auto a = std::minmax_element(qef_amp_table.begin(), qef_amp_table.end());
            printf("[DOPPLER] beam gain: -3 dB ground radius %.1f km, HPBW %.3f deg (ka %.2f), power offset %.1f dB, "
                   "DL gain %.1f .. %.1f dB over the window\n",
                   beam_km, 2.0 * beam_half_hpbw(beam_km * 1000.0) * 180.0 / M_PI, ka, offset_db,
                   20.0 * log10(fmax(*a.first, 1e-15)), 20.0 * log10(*a.second));
        }
        return true;
    }

    // Value of a QEF row vector at experiment time t, interpolated between rows;
    // the first / last row is held outside the table
    double doppler_shift::qef_at(const std::vector<double>& v, double t) {
        const double x = (t - qef_t0) / qef_dt;
        if (x <= 0.0) return v.front();
        if (x >= (double)(v.size() - 1)) return v.back();
        const size_t i = (size_t)x;
        return v[i] + (x - (double)i) * (v[i + 1] - v[i]);
    }
}
