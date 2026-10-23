/*
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

#include "srslte/common/log.h"
#include "srslte/common/threads.h"
#include "srslte/srslte.h"

#include "sonica_enb/hdr/phy/sf_worker.h"

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


// Enable this to log SI
//#define LOG_THIS(a) 1

// Enable this one to skip SI-RNTI
#define LOG_THIS(rnti) (rnti != 0xFFFF)


using namespace asn1::rrc;

//#define DEBUG_WRITE_FILE

namespace sonica_enb {

sf_worker::~sf_worker()
{
  sonica_enb_dl_nbiot_free(&enb_dl);
  sonica_enb_ul_nbiot_free(&enb_ul);

  for (int p = 0; p < SRSLTE_MAX_PORTS; p++) {
    if (signal_buffer_rx[p]) {
      free(signal_buffer_rx[p]);
    }
    if (signal_buffer_tx[p]) {
      free(signal_buffer_tx[p]);
    }
  }
}

void sf_worker::init(phy_common* phy_, srslte::log* log_h_)
{
  phy   = phy_;
  log_h = log_h_;
  worker_init(phy, log_h);

  initiated = true;
  running   = true;
}

void sf_worker::worker_init(phy_common* phy_, srslte::log* log_h_)
{
  reset();

  srslte_nbiot_cell_t cell = phy->get_cell();
  uint32_t nof_prb = phy->get_nof_prb();
  uint32_t sf_len  = SRSLTE_SF_LEN_PRB(nof_prb);

  for (uint32_t p = 0; p < phy->get_nof_ports(); p++) {
    signal_buffer_rx[p] = srslte_vec_cf_malloc(2 * sf_len);
    if (!signal_buffer_rx[p]) {
      ERROR("Error allocating memory\n");
      return;
    }
    srslte_vec_cf_zero(signal_buffer_rx[p], 2 * sf_len);
    signal_buffer_tx[p] = srslte_vec_cf_malloc(2 * sf_len);
    if (!signal_buffer_tx[p]) {
      ERROR("Error allocating memory\n");
      return;
    }
    srslte_vec_cf_zero(signal_buffer_tx[p], 2 * sf_len);
  }
  if (sonica_enb_dl_nbiot_init(&enb_dl, signal_buffer_tx)) {
    ERROR("Error initiating ENB DL\n");
    return;
  }
  if (sonica_enb_ul_nbiot_init(&enb_ul, signal_buffer_rx[0])) {
    ERROR("Error initiating ENB UL\n");
    return;
  }
  if (sonica_enb_dl_nbiot_set_cell(&enb_dl, cell)) {
    ERROR("Error initiating ENB DL\n");
    return;
  }
  if (sonica_enb_ul_nbiot_set_cell(&enb_ul, cell)) {
    ERROR("Error initiating ENB UL\n");
    return;
  }
}

void sf_worker::reset()
{
  initiated = false;
  // ue_db.clear();
}


cf_t* sf_worker::get_buffer_rx(uint32_t antenna_idx)
{
  return signal_buffer_rx[antenna_idx];
}

cf_t* sf_worker::get_buffer_tx(uint32_t antenna_idx)
{
  return signal_buffer_tx[antenna_idx];
}

void sf_worker::set_time(uint32_t hfn_, uint32_t tti_, uint32_t tx_worker_cnt_, srslte_timestamp_t tx_time_)
{
  hfn       = hfn_;
  tti_rx    = tti_;
  tti_tx_dl = TTI_ADD(tti_rx, FDD_HARQ_DELAY_UL_MS);
  tti_tx_ul = TTI_UL_RX_NB(tti_rx);

  t_tx_dl = TTIMOD(tti_tx_dl);
  t_rx    = TTIMOD(tti_rx);
  t_tx_ul = TTIMOD(tti_tx_ul);

  tx_worker_cnt = tx_worker_cnt_;
  srslte_timestamp_copy(&tx_time, &tx_time_);
}

void sf_worker::work_imp()
{
  std::lock_guard<std::mutex> lock(work_mutex);

  log_h->step(tti_rx);

  srslte::rf_buffer_t tx_buffer = {};
  for (uint32_t ant = 0; ant < phy->get_nof_ports(); ant++) {
    tx_buffer.set(0, ant, phy->get_nof_ports(), get_buffer_tx(ant));
  }

  if (!running) {
    phy->worker_end(this, tx_buffer, 0, tx_time);
    return;
  }

  stack_interface_phy_nb* stack = phy->stack;

  // Downlink grants to transmit this TTI
  stack_interface_phy_nb::dl_sched_list_t dl_grants(1);

  if (stack->get_dl_sched(hfn, tti_tx_dl, dl_grants) < 0) {
    Error("Getting DL scheduling from MAC\n");
    phy->worker_end(this, tx_buffer, 0, tx_time);
    return;
  }

  // Uplink grants to receive this TTI
  stack_interface_phy_nb::ul_sched_list_t ul_grants = phy->get_ul_grants(t_rx);
  // Uplink grants to transmit this tti and receive in the future
  stack_interface_phy_nb::ul_sched_list_t ul_grants_tx = phy->get_ul_grants(t_tx_ul);

  // Get UL scheduling for the TX TTI from MAC
  if (stack->get_ul_sched(hfn, tti_tx_ul, ul_grants_tx) < 0) {
    Error("Getting UL scheduling from MAC\n");
    printf("Getting UL scheduling from MAC Error\n");
    phy->worker_end(this, tx_buffer, 0, tx_time);
    return;
  }

  if(dl_grants[0].nof_grants > 0 && dl_grants[0].npdsch.has_npdcch){
    printf("TTI %d: DL grant available. has_npdcch. nof_grants=%d\n", tti_tx_dl, dl_grants[0].nof_grants);
    printf("TTI %d: DL tti_rx=%d, tti_tx_ul=%d \n", tti_tx_dl, tti_rx, tti_tx_ul);
    uint8_t* dl_data = dl_grants[0].npdsch.data[0];

    uint32_t test_print_len = 50;
    printf("PHY: sf_worker------------DL DATA Start: length=%d\n", test_print_len);
    for(uint32_t i =0; i<test_print_len; i++){
      printf("0x%02x ", dl_data[i]);
    }
    printf("\nPHY: sf_worker------------DL DATA End\n");

  }

  if(ul_grants_tx[0].nof_grants > 0){
    printf("TTI %d: UL grant available. nof_grants=%d\n", tti_rx, ul_grants_tx[0].nof_grants);
    printf("TTI %d: t_rx=%d, t_tx_ul=%d \n", tti_rx, t_rx, t_tx_ul);

    if (srslte_ra_nbiot_ul_dci_to_grant(&ul_grants_tx[0].npusch[0].dci.ra_dci, &ugrant,
                                        tti_tx_dl,
                                        SRSLTE_NPUSCH_SC_SPACING_15000)) {
      Error("Failed to generate Grant from DCI");
    } else {
      npusch_rnti = ul_grants_tx[0].npusch[0].dci.rnti;
      npusch_start = ugrant.tx_tti;
      npusch_data  = ul_grants_tx[0].npusch[0].data;
      //sonica_enb_ul_nbiot_cfg_grant(&enb_ul, &ugrant, npusch_rnti);
      printf("  Recording grants for R%x, G%d=%d\n", npusch_rnti, ugrant.tx_tti, tti_tx_ul);
      has_ugrant = true;
    }
  }

  // For test: should receive UL data if ul grants > 0
  if (ul_grants[0].nof_grants > 0) {
    printf("UL TTI %d: ul_grants = %d \n", tti_rx, ul_grants[0].nof_grants);
  }

  work_ul(ul_grants);

  // Save grants
  phy->set_ul_grants(t_tx_ul, ul_grants_tx);
  phy->set_ul_grants(t_rx, ul_grants);

  work_dl(dl_grants[0], ul_grants_tx[0]);

  // TODO
  phy->worker_end(this, tx_buffer, SRSLTE_SF_LEN_PRB(phy->get_nof_prb()), tx_time);
}


static void log_data(uint8_t *data, uint32_t len)
{
  printf("Data len = %u\n", len);
  for (uint32_t i = 0; i < len && i < 256; ) {
    for (uint32_t j = 0; j < 16 && i < len; j++, i++) {
      printf(" %02x", data[i]);
    }
    printf("\n");
  }
  printf("\n");
}

void sf_worker::work_ul(stack_interface_phy_nb::ul_sched_list_t ul_grants)
{
  uint32_t sfn = tti_rx / 10;
  uint32_t sf_idx = tti_rx % 10;

  if (has_ugrant && tti_rx == npusch_start) {
    printf("RX %d.%d Activivating NPUSCH for RNTI %x\n", sfn, sf_idx, npusch_rnti);
    sonica_enb_ul_nbiot_cfg_grant(&enb_ul, &ugrant, npusch_rnti);
  }

  if (enb_ul.has_ul_grant) {
    int ret = sonica_enb_ul_nbiot_decode_npusch(&enb_ul, signal_buffer_rx[0], sf_idx, npusch_data);
    printf("RX %d.%d decoding result %d\n", sfn, sf_idx, ret);
    int len = enb_ul.npusch_cfg.grant.mcs.tbs / 8;
    if (ret == SRSLTE_SUCCESS) {
      log_data(npusch_data, len);
      phy->stack->crc_info(npusch_start, npusch_rnti, len, true);
    }
    if (ret == -1) {
      // report a failed NPUSCH decode as a CRC failure
      printf("PHY: NPUSCH decode FAILED rnti=%x len=%d -> CRC fail\n", npusch_rnti, len);
      phy->stack->crc_info(npusch_start, npusch_rnti, len, false);
    }
    if (ret == SRSLTE_SUCCESS || ret == -1) {
      has_ugrant = false;
    }
  }
}

// TODO: Assign DCI with HARQ in MAC
static bool h_ndi_dl = false;
static bool h_ndi_ul = false;

/* True if this subframe carries SIB1-NB, as scheduled by bc_sched::is_sib1_sf(). These subframes
 * cannot carry another NPDSCH (36.213 16.4.1.3). */
static bool sf_is_sib1(uint32_t sfn, uint32_t sf_idx)
{
  if (sf_idx != 4) {
    return false;
  }
  srslte_mib_nb_t mib_nb  = {};
  mib_nb.sched_info_sib1  = 5; // same value as bc_sched
  mib_nb.sys_info_tag     = 14;
  mib_nb.ac_barring       = false;
  mib_nb.mode             = SRSLTE_NBIOT_MODE_STANDALONE;

  uint32_t num_rep    = srslte_ra_n_rep_sib1_nb(&mib_nb);
  uint32_t sib1_start = srslte_ra_nbiot_get_starting_sib1_frame(99, &mib_nb);

  for (uint32_t k = 0; k < 4; k++) {
    for (uint32_t i = 0; i < num_rep; i++) {
      uint32_t first = k * SIB1_NB_TTI + sib1_start + (i * SIB1_NB_TTI / num_rep);
      uint32_t valid_si_sfn = sfn + srslte_ra_nbiot_sib1_start(99, &mib_nb);
      if (sfn >= first && sfn < first + 16 && (valid_si_sfn % 2) == 0) {
        return true;
      }
    }
  }
  return false;
}

void sf_worker::work_dl(stack_interface_phy_nb::dl_sched_t& dl_grants,
                        stack_interface_phy_nb::ul_sched_t& ul_grants_tx)
{
  srslte_nbiot_cell_t cell = phy->get_cell();
  uint32_t sfn = tti_tx_dl / 10;
  uint32_t sf_idx = tti_tx_dl % 10;
  bool has_sib1_cfg = false;

  // First, put NPSS/NSSS/Ref/NPBCH in
  sonica_enb_dl_nbiot_put_base(&enb_dl, hfn, tti_tx_dl);


  if (ul_grants_tx.nof_grants > 0 && ul_grants_tx.npusch[0].needs_npdcch) {
    srslte_nbiot_dci_ul_t *udci = &ul_grants_tx.npusch[0].dci;
    udci->ra_dci.ndi = h_ndi_ul;
    h_ndi_ul = !h_ndi_ul;
    sonica_enb_dl_nbiot_put_npdcch_ul(&enb_dl, &udci->ra_dci, udci->rnti, sf_idx);
    printf("PHY UL: TTI %d, sending UL DCI to RNTI %d\n", tti_tx_dl, udci->rnti);
  }

  if (dl_grants.nof_grants > 0) {
    srslte_ra_nbiot_dl_dci_t *dci = &dl_grants.npdsch.dci;

    // First case: SIB 1 needs to be sent out immediately at highest priority
    if (dci->alloc.has_sib1) {
      // SIB1 hasn't been configured, expecting a new config from MAC
      if (dl_grants.npdsch.is_new_sib) {
        if (sib1_npdsch_cfg.sf_idx > 0) {
          Warning("Original SIB1 config has not finished transmission\n");
        }

        srslte_ra_nbiot_dl_grant_t sib1_grant;

        srslte_ra_nbiot_dl_dci_to_grant(dci, &sib1_grant, sfn, sf_idx,
                                        DUMMY_R_MAX, true, cell.mode);
        if (srslte_npdsch_cfg(&sib1_npdsch_cfg, cell, &sib1_grant, sf_idx)) {
          Error("Error configuring NPDSCH for SIB1\n");
          return;
        }

        has_sib1_cfg = true;
      } else {
        if (sib1_npdsch_cfg.sf_idx > 0) {
          has_sib1_cfg = true;
        } else {
          Warning("New SIB1 configuration missing\n");
        }
      }

      // Only send out SIB1 when we actually have that config
      if (has_sib1_cfg) {
        if (sonica_enb_dl_nbiot_put_npdsch(&enb_dl, &sib1_npdsch_cfg,
                                           dl_grants.npdsch.data[0],
                                           dci->alloc.rnti)) {
          ERROR("Error encoding NPDSCH for SIB1\n");
        }

        // Warning("SIB1 %d @ %d\n", sib1_npdsch_cfg.sf_idx, tti_tx_dl);

        if (sib1_npdsch_cfg.sf_idx == sib1_npdsch_cfg.grant.nof_sf * sib1_npdsch_cfg.grant.nof_rep) {
          bzero(&sib1_npdsch_cfg, sizeof(srslte_npdsch_cfg_t));
        }
      }
    } else {
      if (!dl_grants.npdsch.has_npdcch) {
        // No NPDCCH but not SIB1, this should be other SIBs, should be immediately sent
        // TODO: check for collisions
        srslte_ra_nbiot_dl_grant_t sib2_grant;

        srslte_ra_nbiot_dl_dci_to_grant(dci, &sib2_grant, sfn, sf_idx,
                                        DUMMY_R_MAX, true, cell.mode);
        if (srslte_npdsch_cfg(&npdsch_cfg, cell, &sib2_grant, sf_idx)) {
          Error("Error configuring NPDSCH for SIB1\n");
        } else {
          npdsch_cfg.has_bcch = true;

          npdsch_active = true;
          npdsch_rnti = dci->alloc.rnti;
          npdsch_data = dl_grants.npdsch.data[0];
        }
      } else {
        // This is normal user data. In this case, transmit DCI immediately and
        // queue the data

        srslte_ra_nbiot_dl_grant_t grant;

        srslte_ra_nbiot_dl_dci_to_grant(dci, &grant, sfn, sf_idx,
                                        DUMMY_R_MAX, false, cell.mode);

        // srslte_ra_nbiot_dl_dci_to_grant() does not skip SIB1-NB subframes: move the start to the
        // next valid DL subframe
        while (sf_is_sib1(grant.start_sfn, grant.start_sfidx) ||
               !srslte_ra_nbiot_is_valid_dl_sf(grant.start_sfn * 10 + grant.start_sfidx)) {
          uint32_t next   = (grant.start_sfn * 10 + grant.start_sfidx + 1) % 10240;
          grant.start_sfn = next / 10;
          grant.start_sfidx = next % 10;
        }

        dl_pending_grants.push_back({ grant, dci->alloc.rnti, dl_grants.npdsch.data[0] });

        if (dci->alloc.rnti == SRSLTE_PRNTI) {
          // Paging: DCI format N2 with P-RNTI, no HARQ
          sonica_enb_dl_nbiot_put_npdcch_paging(&enb_dl, dci, sf_idx);
          printf("PHY DL: TTI %d, sending PAGING DCI (format N2, P-RNTI)\n", tti_tx_dl);
        } else {
          dci->ndi = h_ndi_dl;
          h_ndi_dl = !h_ndi_dl;

          sonica_enb_dl_nbiot_put_npdcch_dl(&enb_dl, dci, sf_idx);
        }
      }
    }
  }

  // Drop grants whose start subframe has passed, so they do not block the queue
  while (!dl_pending_grants.empty()) {
    const srslte_ra_nbiot_dl_grant_t& g   = dl_pending_grants.front().grant;
    uint32_t                          st  = g.start_sfn * 10 + g.start_sfidx;
    uint32_t                          age = (tti_tx_dl + 10240 - st) % 10240;
    if (age > 0 && age < 1024) {
      printf("PHY DL: dropping stale grant for rnti %d (start TTI %d, now %d)\n",
             dl_pending_grants.front().rnti, st, tti_tx_dl);
      dl_pending_grants.pop_front();
    } else {
      break;
    }
  }

  if (!dl_pending_grants.empty()) {
    dl_grant_record& head_grant = dl_pending_grants.front();
    if (npdsch_active && npdsch_rnti == SRSLTE_SIRNTI &&
        head_grant.grant.start_sfn == sfn && head_grant.grant.start_sfidx == sf_idx) {
      // A due grant takes precedence over an ongoing SI transmission, which is repeated in its next
      // window
      printf("PHY DL: preempting SI NPDSCH for grant to rnti %d\n", head_grant.rnti);
      bzero(&npdsch_cfg, sizeof(srslte_npdsch_cfg_t));
      npdsch_active = false;
    }
    if (!npdsch_active &&
        head_grant.grant.start_sfn == sfn &&
        head_grant.grant.start_sfidx == sf_idx) {
      if (srslte_npdsch_cfg(&npdsch_cfg, cell, &head_grant.grant, sf_idx)) {
        Error("Error configuring NPDSCH for Data\n");
      } else {
        npdsch_active = true;
        npdsch_rnti = head_grant.rnti;
        npdsch_data = head_grant.data;
      }
      dl_pending_grants.pop_front();
    }
  }

  if (!has_sib1_cfg && npdsch_active && srslte_ra_nbiot_is_valid_dl_sf(tti_tx_dl)) {
    if (sonica_enb_dl_nbiot_put_npdsch(&enb_dl, &npdsch_cfg,
                                       npdsch_data, npdsch_rnti)) {
      ERROR("Error encoding NPDSCH\n");
    }
    printf("S %d.%d => %d\n", sfn, sf_idx, npdsch_rnti);
    if (npdsch_rnti != SRSLTE_SIRNTI) {
      log_data(npdsch_data, 7);
    }

    if (npdsch_cfg.num_sf == npdsch_cfg.grant.nof_sf * npdsch_cfg.grant.nof_rep) {
      bzero(&npdsch_cfg, sizeof(srslte_npdsch_cfg_t));
      npdsch_active = false;
    }
  }

  sonica_enb_dl_nbiot_gen_signal(&enb_dl);
}

} // namespace sonica_enb