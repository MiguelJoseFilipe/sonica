#!/bin/bash
# Copyright 2025-2026 LAAS-CNRS
# SPDX-License-Identifier: AGPL-3.0-or-later
#
# Entrypoint of the Sonica NB-IoT image, modelled on OAI's gNB entrypoint:
#   1. make sure the USRP FPGA images are present (downloaded on first run, cached in a volume)
#   2. seed /opt/sonica/etc from the shipped example configs if it is empty
#   3. exec the requested program so signals and exit codes reach tini
#
# USAGE (docker run / compose command):
#   enb                 -> sonica_enb -c /opt/sonica/etc/enb_nb.conf   (default)
#   epc                 -> srsepc_ciot -c /opt/sonica/etc/epc.conf
#   probe               -> uhd_find_devices + uhd_usrp_probe (radio check, no traffic)
#   shell               -> interactive bash
#   <anything else>     -> executed as-is (e.g. "sonica_enb --help")
# ENV:
#   ENB_CONF / EPC_CONF   config paths (default /opt/sonica/etc/{enb_nb,epc}.conf)
# The configs reference companion files by bare name (sib_nb.conf, rr_nb.conf, drb_nb.conf, user_db.csv).
# Sonica resolves those relative to the working directory or to /etc/sonica, so the entrypoint cds into $ETC
# and the image symlinks /etc/sonica -> /opt/sonica/etc. Both binaries take the config as a POSITIONAL arg.
#   EXTRA_ARGS            appended to the sonica_enb / srsepc_ciot command line
#   UHD_IMAGE_TYPES       which FPGA/firmware images to fetch (default b2xx = USRP B200/B210; use "all" or x3xx...)
#   SKIP_UHD_IMAGES=1     do not attempt to download USRP images at start
set -e
ETC=/opt/sonica/etc
ENB_CONF=${ENB_CONF:-$ETC/enb_nb.conf}
EPC_CONF=${EPC_CONF:-$ETC/epc.conf}

if [ -d "$ETC" ] && [ -z "$(ls -A "$ETC" 2>/dev/null)" ]; then
  echo "== seeding $ETC with the example configuration"
  cp -a /opt/sonica/etc-default/. "$ETC"/
fi

if [ "${SKIP_UHD_IMAGES:-0}" != "1" ] && [ "$1" != "shell" ]; then
  echo "== USRP images (${UHD_IMAGES_DIR:-/usr/local/share/uhd/images}, types=${UHD_IMAGE_TYPES:-b2xx})"
  uhd_images_downloader -t "${UHD_IMAGE_TYPES:-b2xx}" 2>&1 | tail -5 || \
    echo "   WARNING: image download failed (offline?) - a cached volume or a pre-flashed USRP is required"
fi

case "$1" in
  enb)   echo "== starting sonica_enb with $ENB_CONF"; cd "$ETC"; exec sonica_enb "$ENB_CONF" $EXTRA_ARGS ;;
  epc)   echo "== starting srsepc_ciot with $EPC_CONF"; cd "$ETC"; exec srsepc_ciot "$EPC_CONF" $EXTRA_ARGS ;;
  probe) uhd_find_devices; exec uhd_usrp_probe ${EXTRA_ARGS:+--args "$EXTRA_ARGS"} ;;
  shell) exec /bin/bash ;;
  *)     exec "$@" ;;
esac
