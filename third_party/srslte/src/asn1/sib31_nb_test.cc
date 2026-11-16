/*
 * SIB31-NB (Rel-17 NTN) and SIB1-NB v1610/v1700 codec check.
 *
 * Golden vectors: UPER BCCH-DL-SCH-Message-NB encodings produced by asn1tools from the ASN.1 of
 * TS 36.331 V17.16.0 (independent of this codec), plus the encodings of Sonica's own SIB1-NB and SI
 * (docker/etc/*_nb.conf) checked with asn1tools. Each vector must decode and re-encode bit-exactly.
 */
#include "srslte/asn1/rrc_asn1_nbiot.h"
#include <cstdio>
#include <string>
#include <vector>

using namespace asn1;
using namespace asn1::rrc;

static const struct {
  const char* name;
  const char* hex;
} vectors[] = {
    {"sv extremes + all optionals", "002147860000003ffffff80000000000ffffdffffffb9e6c00007327fffe7ffff800"},
    {"sv realistic, no optionals", "002145007316b662462e78e60e3cba7ba22e386a00000000"},
    {"orbital extremes", "002146c3ffffffffffffffffffff80000000000000fffffff5ff97ac050000"},
    {"orbital mid, kmac", "002146c700353d2c80006000c35000f42407ffffff000000118000022ff000"},
    {"sib31 + sib32 (opaque, empty)", "006145008000006000002800000e00048001600060000021804000"},
    {"SIB1-NB with v1530 sched (sibType31) + v1700 cellBarred-NTN", "401000800806000e000020200e030d08a1562132"},
    {"SI today (SIB2+SIB3)", "00402b517db7c0250c9b1c0c95b1cd81db1d650f081b0762ffb0dc080000071004005d00"},
    {"B: SIB1 NTN (2 SI messages)", "401000800806000e0000a040a0042d08ad020ab284c8"},
};

#define CHECK(c)                                                                                                       \
  if (!(c)) {                                                                                                          \
    printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c);                                                               \
    return 1;                                                                                                          \
  }

static std::vector<uint8_t> from_hex(const std::string& h)
{
  std::vector<uint8_t> b(h.size() / 2);
  for (size_t i = 0; i < b.size(); i++) {
    b[i] = (uint8_t)std::stoi(h.substr(2 * i, 2), nullptr, 16);
  }
  return b;
}

static int round_trip(const char* hex, bcch_dl_sch_msg_nb_s& m)
{
  std::vector<uint8_t> in = from_hex(hex);
  cbit_ref             rd(in.data(), in.size());
  CHECK(m.unpack(rd) == SRSASN_SUCCESS);
  uint8_t out[256] = {};
  bit_ref wr(out, sizeof(out));
  CHECK(m.pack(wr) == SRSASN_SUCCESS);
  CHECK((size_t)wr.distance_bytes() == in.size());
  CHECK(std::equal(in.begin(), in.end(), out));
  return 0;
}

int main()
{
  for (const auto& v : vectors) {
    bcch_dl_sch_msg_nb_s m;
    if (round_trip(v.hex, m)) {
      printf("vector: %s\n", v.name);
      return 1;
    }
  }

  // Field values of the extreme state-vector case
  bcch_dl_sch_msg_nb_s m;
  CHECK(round_trip(vectors[0].hex, m) == 0);
  const serving_satellite_info_r17_s& s =
      m.msg.c1().sys_info_r13().crit_exts.sys_info_r13().sib_type_and_info_r13[0].sib31_v1700().serving_satellite_info_r17;
  CHECK(s.ephemeris_info_r17.type() == serving_satellite_info_r17_s::ephemeris_info_r17_c_::types::state_vectors);
  CHECK(s.ephemeris_info_r17.state_vectors.pos_x_r17 == -33554432);
  CHECK(s.ephemeris_info_r17.state_vectors.pos_y_r17 == 33554431);
  CHECK(s.ephemeris_info_r17.state_vectors.velocity_vx_r17 == -131072);
  CHECK(s.nta_common_params_r17.nta_common_drift_r17 == -261935);
  CHECK(s.ul_sync_validity_dur_r17.to_number() == 900);
  CHECK(s.epoch_time_r17_present && s.epoch_time_r17.start_sfn_r17 == 1023 && s.epoch_time_r17.start_sub_frame_r17 == 9);
  CHECK(s.k_mac_r17_present && s.k_mac_r17 == 512);

  // 33-bit semiMajorAxis (packed in two chunks)
  CHECK(round_trip(vectors[2].hex, m) == 0);
  CHECK(m.msg.c1().sys_info_r13().crit_exts.sys_info_r13().sib_type_and_info_r13[0].sib31_v1700()
            .serving_satellite_info_r17.ephemeris_info_r17.orbital_params.semi_major_axis_r17 == 8589934591ULL);

  printf("sib31_nb_test: %zu vectors OK\n", sizeof(vectors) / sizeof(vectors[0]));
  return 0;
}
