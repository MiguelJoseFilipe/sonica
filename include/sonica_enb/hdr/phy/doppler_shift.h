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

namespace sonica_enb {
    class doppler_shift
    {
      public:
      static double t_v(double x);
      static double get_doppler_effect(double Theta_max, double t);
      static std::tuple<int, double, float> get_parameters();
      static double get_experiment_time(double visbility_time, double start_time);
      static double get_fspl();
      std::string get_remote_ip();
      std::vector<double> precompute_doppler(double Theta_max, double t_start, double t_end, double dt);
      static double get_precomputed_doppler(const std::vector<double>& doppler_values,
                               double t_start, double dt, double t);
    };
}
#endif /* DOPPLER_SHIFT_H */
