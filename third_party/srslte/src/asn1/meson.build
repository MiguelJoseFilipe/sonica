srslte_asn1_srcs = files([
  'liblte_common.cc',
  'liblte_mme.cc',
  'liblte_m2ap.cc',
  'gtpc.cc'
])

srslte_asn1 = static_library('srslte_asn1', srslte_asn1_srcs,
  include_directories : srslte_inc
)

srslte_asn1_utils = static_library('srslte_asn1_utils', 'asn1_utils.cc',
  include_directories : srslte_inc,
  cpp_args : ['-Os'],
  link_with : srslte_common
)

rrc_asn1_srcs = files([
  'rrc_asn1.cc',
  'rrc_asn1_enum.cc',
  'rrc_asn1_nbiot.cc',
  'rrc_asn1_utils.cc'
])

srslte_rrc_asn1 = static_library('srslte_rrc_asn1', rrc_asn1_srcs,
  include_directories : srslte_inc,
  cpp_args : ['-Os'],
  link_with : [srslte_common, srslte_asn1_utils]
)

srslte_s1ap_asn1 = static_library('srslte_s1ap_asn1', 's1ap_asn1.cc',
  include_directories : srslte_inc,
  cpp_args : ['-Os'],
  link_with : [srslte_common, srslte_asn1_utils]
)

# SIB31-NB / SIB1-NB Rel-17 extensions: golden vectors from the TS 36.331 ASN.1 (meson test)
test('sib31_nb', executable('sib31_nb_test', 'sib31_nb_test.cc',
  include_directories : srslte_inc,
  link_with : [srslte_rrc_asn1, srslte_asn1_utils, srslte_common]))
