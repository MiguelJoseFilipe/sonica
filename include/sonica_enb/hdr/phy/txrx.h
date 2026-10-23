/*
 * Copyright 2013-2020 Software Radio Systems Limited
 * Copyright 2021      Metro Group @ UCLA
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

#ifndef SRSENB_NB_TXRX_H
#define SRSENB_NB_TXRX_H

#include "phy_common.h"
#include "nprach_worker.h"
#include "srslte/common/log.h"
#include "srslte/common/thread_pool.h"
#include "srslte/common/threads.h"
#include "srslte/config.h"
#include "srslte/phy/channel/channel.h"
#include "srslte/radio/radio.h"

#include <atomic>
#include <chrono>
#include <mutex>

// Experiment clock: pass time received from the clock server at the last
// synchronization, and local wall-clock time of that synchronization (s)
struct ExperimentTimes {
  double experiment_time;
  double start_time;
};
extern ExperimentTimes experiment_times;
extern std::mutex experiment_times_mutex;

// The pass clock is anchored at the first transmitted subframe and re-synchronized
// with the clock server; sf_worker extrapolates it and loops over the pass.
extern std::atomic<bool> g_tx_started;  // set at the first transmitted subframe
extern double g_pass_start_offset;      // -visibility_time/2: start of the pass and of the Doppler table
extern double g_vis_time;               // visibility time (loop period)

// Calibration ("Calibration Mode = 1" in config.txt): transmit a constant Doppler
// ("Calibration Doppler Hz", 0 by default) instead of the orbital curve.
extern std::atomic<bool> g_calibration;
extern std::atomic<double> g_calibration_doppler_hz;

namespace sonica_enb {

class txrx final : public srslte::thread
{
public:
  txrx();
  bool init(srslte::radio_interface_phy* radio_handler,
            srslte::thread_pool*         _workers_pool,
            phy_common*                  worker_com,
            nprach_worker*               nprach_,
            srslte::log*                 log_h,
            uint32_t                     prio);
  void stop();

private:
  void run_thread() override;

  srslte::radio_interface_phy* radio_h      = nullptr;
  srslte::log*                 log_h        = nullptr;
  srslte::thread_pool*         workers_pool = nullptr;
  nprach_worker*               nprach       = nullptr;
  phy_common*                  worker_com   = nullptr;

  // Main system HFN & TTI counter
  uint32_t hfn = 0;
  uint32_t tti = 0;

  uint32_t tx_worker_cnt = 0;
  uint32_t nof_workers   = 0;
  bool     running       = false;
};

} // namespace sonica_enb

#endif // SRSENB_NB_TXRX_H
