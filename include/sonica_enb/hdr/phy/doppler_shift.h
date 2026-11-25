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

#include <tuple>
#include <atomic>
#include <string>
#include <vector>
#ifndef DOPPLER_SHIFT_H
#define DOPPLER_SHIFT_H
extern int* pointer_maxElevation;
extern std::vector<double> doppler_table;
extern std::atomic<double> experiment_time;
extern std::atomic<double> start_time;

extern double* pointer_AWGN;
extern double* pointer_FSPL;

// Quasi-earth-fixed model ("Doppler model = 1" in config.txt), ported from the OAI gNB
// (nr_doppler_shift.c): residual Doppler and DL beam gain played back from a table.
extern bool qef_model;                        // true when the QEF table drives doppler_table
extern std::vector<double> qef_amp_table;     // DL amplitude per row, empty = beam gain off
extern double qef_t0;                         // experiment time of row 0 (s)
extern double qef_dt;                         // row spacing (s)
extern std::vector<double> qef_sat_table[6];  // satellite ECEF x, y, z (m), vx, vy, vz (m/s) per row
extern std::vector<double> qef_range_table;   // satellite-device range (m) per row, for the propagation delay

namespace sonica_enb {
    class doppler_shift
    {
      public:
      static double t_v(double x);
      static double get_doppler_effect(double Theta_max, double t);
      static double get_delay(double Theta_max, double t);  // earth-moving one-way delay (s), as OAI ds_delay_at
      static std::tuple<int, double, float> get_parameters();
      static double get_experiment_time(double visbility_time, double start_time);
      static double get_fspl();
      std::string get_remote_ip();
      std::vector<double> precompute_doppler(double Theta_max, double t_start, double t_end, double dt);
      static double get_precomputed_doppler(const std::vector<double>& doppler_values,
                               double t_start, double dt, double t);
      static bool load_qef(double carrier_hz);
      static double qef_at(const std::vector<double>& v, double t);
    };

    // Satellite ECEF state from the QEF table at the on-air time of DL subframe tti_tx (0..10239), on the
    // pass clock of the Doppler; false (state of the first row) before the first transmitted subframe or in
    // calibration, zeros without a QEF table. Defined in txrx.cc; used by RRC for SIB31-NB.
    bool ntn_sat_state_at(uint32_t tti_tx, double pos[3], double vel[3]);
}
#endif /* DOPPLER_SHIFT_H */
