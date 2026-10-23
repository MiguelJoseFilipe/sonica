# Sonica NB-IoT in Docker (USRP B210)

Container build and deployment of the terrestrial NB-IoT cell measured on the NOVA platform: the
Sonica eNodeB and CIoT EPC of this branch, with a USRP B210.

| Stage (in `Dockerfile.sonica.ubuntu`) | Content |
|---|---|
| `sonica-base` | Ubuntu 22.04 + UHD 4.8.0.0 built from source with OAI's TDD switching patch (`uhd-4.5plus-tdd-patch.diff`, from OAI `cmake_targets/tools/`), plus `tini` |
| `sonica-build` | Sonica compiled with meson/ninja against that UHD |
| `sonica` (final) | Runtime only: UHD libraries and tools, the two Sonica binaries, example configs, entrypoint, network tools (`ping`, `iperf3`, `tcpdump`, `ip`) |

## Build

From the root of this repository:

```bash
docker build -f docker/Dockerfile.sonica.ubuntu -t sonica-nbiot:latest .
```

The first build takes 15 to 25 min (UHD from source); the UHD stage is then cached, so a Sonica code
change rebuilds in about a minute. Options: `--build-arg UHD_VERSION=4.8.0.0`,
`--build-arg UBUNTU_IMAGE=ubuntu:22.04`.

## Run

```bash
cd docker
docker compose run --rm sonica-enb probe     # radio check: uhd_find_devices + uhd_usrp_probe
docker compose up -d sonica-epc              # CIoT EPC (creates its TUN interface)
docker compose up -d sonica-enb              # eNodeB
docker compose logs -f sonica-enb
docker compose down
```

- The first start downloads the B2xx FPGA/firmware images into the `uhd-images` volume.
- `etc/` holds the configuration used for the measurements: band 20 standalone carrier
  (`rr_nb.conf`), PLMN 001/01, B210 RF settings (`enb_nb.conf`), paging repetitions (`sib_nb.conf`).
  `user_db.csv` contains the upstream test subscribers: replace them with your SIM cards, and set
  the APN of `epc.conf` to the one configured in the modems.
- Host networking keeps the stock Sonica addressing (eNB and EPC over `127.0.1.x`).
- The container runs privileged with `/dev/bus/usb`: UHD loads the B210 firmware, the device
  re-enumerates under a new USB node, and UHD must reopen it.

## Entrypoint commands

| Command | Action |
|---|---|
| `enb` (default) | `sonica_enb $ENB_CONF` (`/opt/sonica/etc/enb_nb.conf`) |
| `epc` | `srsepc_ciot $EPC_CONF` (`/opt/sonica/etc/epc.conf`) |
| `probe` | `uhd_find_devices` then `uhd_usrp_probe` |
| `shell` | interactive bash inside the image |
| anything else | run as-is, e.g. `docker compose run --rm sonica-enb sonica_enb --help` |

Environment: `ENB_CONF`, `EPC_CONF`, `EXTRA_ARGS` (appended to the command line), `UHD_IMAGE_TYPES`,
`SKIP_UHD_IMAGES`, `SONICA_TA_LIST` (eNB), `SONICA_PAGE_IMSI` (EPC).
