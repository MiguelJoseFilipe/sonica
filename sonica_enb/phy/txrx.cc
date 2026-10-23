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

#include <unistd.h>
#include <chrono>
#include <thread>
#include <cmath>
#include <iostream>
#include <fstream>
#include <ctime>
#include <arpa/inet.h>
#include "srslte/common/log.h"
#include "srslte/common/threads.h"
#include "srslte/srslte.h"

#include "sonica_enb/hdr/phy/sf_worker.h"
#include "sonica_enb/hdr/phy/txrx.h"
#include "sonica_enb/hdr/phy/doppler_shift.h"
double f_carrier = 2154.8975e6;

ExperimentTimes experiment_times{0.0, 0.0};
#include <mutex>
std::mutex experiment_times_mutex;
std::vector<double> doppler_table;
std::atomic<bool> server_connect{false};

// Pass clock state (see txrx.h)
std::atomic<bool> g_tx_started{false};
double g_pass_start_offset = 0.0;
double g_vis_time = 0.0;
std::atomic<bool> g_calibration{false};
std::atomic<double> g_calibration_doppler_hz{0.0};

int* pointer_maxElevation;
double* pointer_AWGN;
double* pointer_FSPL;

#define Error(fmt, ...)                                                                                                \
  if (SRSLTE_DEBUG_ENABLED)                                                                                            \
  log_h->error(fmt, ##__VA_ARGS__)
#define Warning(fmt, ...)                                                                                              \
  if (SRSLTE_DEBUG_ENABLED)                                                                                            \
  log_h->warning(fmt, ##__VA_ARGS__)
#define Info(fmt, ...)                                                                                                 \
  if (SRSLTE_DEBUG_ENABLED)                                                                                            \
  log_h->info(fmt, ##__VA_ARGS__)
#define Debug(fmt, ...)                                                                                                \
  if (SRSLTE_DEBUG_ENABLED)                                                                                            \
  log_h->debug(fmt, ##__VA_ARGS__)

using namespace std;
namespace sonica_enb {
  class TCPClient {
    private:
        int sockfd;
        std::string host;
        int port;
        std::string identifier;

    public:
        TCPClient(const std::string& host, int port, const std::string& identifier = "sonica")
            : sockfd(-1), host(host), port(port), identifier(identifier) {}

        bool connectToServer() {
            sockfd = socket(AF_INET, SOCK_STREAM, 0);
            if (sockfd < 0) {
                perror("Socket creation failed");
                return false;
            }

            sockaddr_in server_addr{};
            server_addr.sin_family = AF_INET;
            server_addr.sin_port = htons(port);

            if (inet_pton(AF_INET, host.c_str(), &server_addr.sin_addr) <= 0) {
                std::cerr << "Invalid address/ Address not supported\n";
                return false;
            }

            if (connect(sockfd, (sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
                perror("Connection Failed");
                return false;
            }

            char buffer[1024] = {0};

            // Receive server prompt
            recv(sockfd, buffer, sizeof(buffer), 0);
            std::cout << "Server: " << buffer << std::endl;

            // Send identifier
            send(sockfd, identifier.c_str(), identifier.length(), 0);

            // Receive confirmation
            memset(buffer, 0, sizeof(buffer));
            recv(sockfd, buffer, sizeof(buffer), 0);
            std::cout << "Server: " << buffer << std::endl;

            return true;
        }

          // Server pass time corrected for the network latency: N probes, keep the
          // one with the smallest round trip and add half of it.
          double queryTime() {
            if (sockfd < 0) {
                std::cerr << "Not connected to server.\n";
                return -1;
            }
            using clk = std::chrono::steady_clock;
            const int N_PROBES = 5;
            bool have_best = false;
            double best_server = 0.0, best_half_rtt = 0.0;
            clk::time_point best_recv{};

            for (int p = 0; p < N_PROBES; ++p) {
                const char* query = "time?";
                auto t0 = clk::now();
                if (send(sockfd, query, strlen(query), 0) < 0) break;
                // 8-byte big-endian double
                double server_t = 0.0;
                uint8_t* dst = reinterpret_cast<uint8_t*>(&server_t);
                int got = 0;
                while (got < 8) {
                    int n = recv(sockfd, dst + got, 8 - got, 0);
                    if (n <= 0) { got = -1; break; }
                    got += n;
                }
                auto t1 = clk::now();
                if (got != 8) break;
                std::reverse(dst, dst + 8);

                double rtt = std::chrono::duration<double>(t1 - t0).count();
                if (!have_best || rtt < 2.0 * best_half_rtt) {   // keep minimum-RTT probe
                    have_best     = true;
                    best_server   = server_t;
                    best_half_rtt = rtt / 2.0;
                    best_recv     = t1;
                }
            }

            if (!have_best) {
                std::cerr << "No response from server or connection closed.\n";
                close(sockfd);
                sockfd = -1;
                return -1;
            }
            double now_adjust = std::chrono::duration<double>(clk::now() - best_recv).count();
            return best_server + best_half_rtt + now_adjust;
        }


        // Ask the server to start the pass clock now (called at the first transmitted subframe)
        void sendStarted() {
            if (sockfd < 0) return;
            const char* msg = "started";
            send(sockfd, msg, strlen(msg), 0);
            char buffer[64] = {0};
            recv(sockfd, buffer, sizeof(buffer), 0);  // consume ack
        }

        void disconnect() {
            if (sockfd >= 0) {
                close(sockfd);
                sockfd = -1;
                std::cout << "Disconnected from server.\n";
            }
        }

        ~TCPClient() {
            disconnect();
        }
    };
    class ServerClockThread : public srslte::thread {
    public:
        ServerClockThread() : srslte::thread("ServerClockThread") {}

        void run_thread() override {
            // Clock server address: REMOTE_IP in config.txt
            std::string server_ip = sonica_enb::doppler_shift().get_remote_ip();
            if (server_ip.empty()) {
                server_ip = "127.0.0.1";
            }
            sonica_enb::TCPClient client(server_ip, 65432);

            if (!client.connectToServer()) {
                std::cerr << "Failed to connect to clock server." << std::endl;
                server_connect = false;
                return;
            }
            server_connect = true;

            // Wait for the first transmitted subframe, then start the server pass clock
            int waited_ms = 0;
            while (!g_tx_started.load() && waited_ms < 120000) {
                usleep(20000);  // 20 ms poll
                waited_ms += 20;
            }
            if (!g_tx_started.load()) {
                std::cerr << "ServerClockThread: eNB never started transmitting within 120s; "
                             "clock NOT anchored (UEs will not be time-aligned)." << std::endl;
                return;
            }
            client.sendStarted();
            // The server starts the clock only once, so a restarted eNB takes the
            // current pass time from the server and rejoins the pass in phase
            {
                double serverT0 = client.queryTime();
                if (!std::isnan(serverT0) && serverT0 != -1.0) {
                    auto now = std::chrono::system_clock::now();
                    double now_sec = std::chrono::duration_cast<std::chrono::duration<double>>(
                                         now.time_since_epoch()).count();
                    std::lock_guard<std::mutex> lock(experiment_times_mutex);
                    experiment_times.experiment_time = serverT0;
                    experiment_times.start_time      = now_sec;
                }
            }
            std::cout << "ServerClockThread: 'started' sent; local pass time taken from server." << std::endl;

            // Periodic re-synchronization with the server clock; sf_worker
            // extrapolates in between. Failed reads are skipped.
            const double RESYNC_SEC = 1.0;
            while (true) {
                usleep((useconds_t)(RESYNC_SEC * 1e6));
                double serverT = client.queryTime();
                if (std::isnan(serverT) || serverT == -1.0) {
                    continue;
                }
                auto now = std::chrono::system_clock::now();
                double now_sec = std::chrono::duration_cast<std::chrono::duration<double>>(
                                     now.time_since_epoch()).count();
                std::lock_guard<std::mutex> lock(experiment_times_mutex);
                experiment_times.experiment_time = serverT;   // server pass-time at this sync
                experiment_times.start_time      = now_sec;   // local epoch at this sync
            }
        }
    };



txrx::txrx() : thread("TXRX")
{
  /* Do nothing */
}

bool txrx::init(srslte::radio_interface_phy* radio_h_,
                srslte::thread_pool*         workers_pool_,
                phy_common*                  worker_com_,
                nprach_worker*               nprach_,
                srslte::log*                 log_h_,
                uint32_t                     prio_)
{
  radio_h       = radio_h_;
  log_h         = log_h_;
  workers_pool  = workers_pool_;
  worker_com    = worker_com_;
  nprach        = nprach_;
  tx_worker_cnt = 0;
  running       = true;

  nof_workers = workers_pool->get_nof_workers();

  start(prio_);
  return true;
}

void txrx::stop()
{
  if (running) {
    running = false;
    wait_thread_finish();
  }
}

void txrx::run_thread()
{
  sf_worker*          worker  = nullptr;
  srslte::rf_buffer_t buffer  = {};
  srslte_timestamp_t  rx_time = {};
  srslte_timestamp_t  tx_time = {};
  uint32_t            sf_len  = SRSLTE_SF_LEN_PRB(worker_com->get_nof_prb());

  float samp_rate = srslte_sampling_freq_hz(worker_com->get_nof_prb());

  // Configure radio
  radio_h->set_rx_srate(samp_rate);
  radio_h->set_tx_srate(samp_rate);

  // Set Tx/Rx frequencies
  double tx_freq_hz = f_carrier;
  double rx_freq_hz = worker_com->get_ul_freq_hz();
  log_h->console(
      "Setting frequency: DL=%.4f Mhz, UL=%.4f MHz", tx_freq_hz / 1e6f, rx_freq_hz / 1e6f);
  radio_h->set_tx_freq(0, tx_freq_hz);
  radio_h->set_rx_freq(0, rx_freq_hz);

  log_h->info("Starting RX/TX thread nof_prb=%d, sf_len=%d\n", worker_com->get_nof_prb(), sf_len);

  // Set TTI so that first TX is at tti=0
  tti = TTI_SUB(0, FDD_HARQ_DELAY_UL_MS + 1);
  hfn = 1023;
  sonica_enb::doppler_shift ds;
  std::tuple<int, double, float> parameters = ds.get_parameters();  int maxElevation = std::get<0>(parameters);  // Access the first value (maxElevation)
  double carrierFrequency = std::get<1>(parameters);  // Access the second value (carrierFrequency)
  double AWGN = std::get<2>(parameters);
  double FSPL = sonica_enb::doppler_shift::get_fspl();
  pointer_AWGN = &AWGN;
  pointer_FSPL = &FSPL;
  pointer_maxElevation = &maxElevation;
  double visibility_time = doppler_shift::t_v(maxElevation);  // Calculate visibility time
  double offset = -visibility_time / 2.0;
  // Pass geometry for sf_worker.cc
  g_pass_start_offset = offset;
  g_vis_time = visibility_time;
  // Calibration: "Calibration Mode" (on/off) and "Calibration Doppler Hz" (constant Doppler)
  {
    std::ifstream cfg("../../../config.txt");
    std::string cline;
    bool calib = false;
    double calib_dopp = 0.0;
    while (std::getline(cfg, cline)) {
      if (cline.find("Calibration Mode =") == 0) {
        std::string v = cline.substr(cline.find('=') + 1);
        v.erase(0, v.find_first_not_of(" \t"));
        try { calib = (std::stoi(v) != 0); } catch (...) {}
      } else if (cline.find("Calibration Doppler Hz =") == 0) {
        std::string v = cline.substr(cline.find('=') + 1);
        v.erase(0, v.find_first_not_of(" \t"));
        try { calib_dopp = std::stod(v); } catch (...) {}
      }
    }
    g_calibration = calib;
    g_calibration_doppler_hz = calib_dopp;
    std::cout << "Calibration Mode: " << (calib ? "ON" : "off")
              << (calib ? (" (constant Doppler " + std::to_string(calib_dopp) + " Hz)") : "")
              << std::endl;
  }
  static ServerClockThread server_clock;
  server_clock.start();
  // Wait for the clock server connection only: the thread runs for the eNB lifetime
  for (int i = 0; i < 200 && !server_connect.load(); ++i) usleep(50000);  // up to 10 s
  if (!server_connect.load()) {
    std::cerr << "WARNING: clock server not connected -- UEs will not be time-aligned." << std::endl;
  }
  // Doppler table over the whole pass [-vis/2, +vis/2]
  doppler_table = ds.doppler_shift::precompute_doppler(maxElevation, offset, visibility_time/2, 0.1);


  // Main loop
  while (running) {

    radio_h->set_tx_freq(0, tx_freq_hz);

    // XXX: Refine this messy logic
    // The HFN passed to SF worker is the Tx HFN
    uint32_t tx_hfn = hfn;
    if (tti >= 10240 - FDD_HARQ_DELAY_UL_MS - 1) {
      tx_hfn = (tx_hfn + 1) % 1024;
      if (tti >= 10240 - 1) {
        hfn = tx_hfn;
      }
    }

    tti    = TTI_ADD(tti, 1);
    worker = (sf_worker*)workers_pool->wait_worker(tti);

    if (worker) {
      // Multiple cell buffer mapping
      for (uint32_t p = 0; p < worker_com->get_nof_ports(); p++) {
        // WARNING: The number of ports for all cells must be the same
        buffer.set(0, p, worker_com->get_nof_ports(), worker->get_buffer_rx(p));
      }

      radio_h->rx_now(buffer, sf_len, &rx_time);

      /* Compute TX time: Any transmission happens in TTI+4 thus advance 4 ms the reception time */
      srslte_timestamp_copy(&tx_time, &rx_time);
      srslte_timestamp_add(&tx_time, 0, FDD_HARQ_DELAY_UL_MS * 1e-3);

      Debug("Setting TTI=%d, tx_mutex=%d, tx_time=%ld:%f to worker %d\n",
            tti,
            tx_worker_cnt,
            tx_time.full_secs,
            tx_time.frac_secs,
            worker->get_id());

      worker->set_time(tx_hfn, tti, tx_worker_cnt, tx_time);
      tx_worker_cnt = (tx_worker_cnt + 1) % nof_workers;

      // Trigger phy worker execution
      worker_com->semaphore.push(worker);
      workers_pool->start_worker(worker);

      // Trigger prach worker execution
      nprach->new_tti(tti, buffer.get(0, 0, worker_com->get_nof_ports()));
    } else {
      // wait_worker() only returns NULL if it's being closed. Quit now to avoid unnecessary loops here
      running = false;
    }
  }
}

} // namespace sonica_enb