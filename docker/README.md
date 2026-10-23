# Sonica NB-IoT with satellite Doppler emulation in Docker (USRP X310)

Container build and deployment of the direct-to-satellite NB-IoT eNodeB of this branch: the Sonica
eNodeB applies the Doppler shift of a LEO pass to its downlink, with the experiment time given by
the clock server of `scripts/ntn_doppler`.

| Stage (in `Dockerfile.sonica.ubuntu`) | Content |
|---|---|
| `sonica-base` | Ubuntu 22.04 + UHD 4.8.0.0 built from source with OAI's TDD switching patch (`uhd-4.5plus-tdd-patch.diff`, from OAI `cmake_targets/tools/`), plus `tini` |
| `sonica-build` | Sonica compiled with meson/ninja against that UHD |
| `sonica` (final) | Runtime only: UHD libraries and tools, the two Sonica binaries, example configs, entrypoint, network tools |

## Build

From the root of this repository, on branch `nova-ntn`:

```bash
docker build -f docker/Dockerfile.sonica.ubuntu -t sonica-ntn:latest .
```

## Run

```bash
python3 scripts/ntn_doppler/TCP_server.py -v      # from scripts/ntn_doppler, reads config.txt there
cd docker
docker compose up -d sonica-ntn-enb
docker compose logs -f sonica-ntn-enb
docker compose down
```

- `../scripts/ntn_doppler/config.txt` sets the pass (maximum elevation), the calibration mode
  (constant Doppler) and the clock server address (`REMOTE_IP`). The eNB reads it as `/config.txt`.
- `etc/` holds the eNB configuration: guard-band carrier (`rr_nb.conf`), PLMN 001/01, X310 RF
  settings (`enb_nb.conf`). No core network is used: `mme_addr` points to an address without an
  MME, and the eNB only broadcasts the cell.
- The transmitted downlink frequency is fixed in `sonica_enb/phy/txrx.cc` (2154.8975 MHz, band 4),
  independently of the EARFCN in `rr_nb.conf`.
- The container `cpuset` is specific to the host used for the measurements; adapt it.
- The X310 address is its factory default (192.168.30.2).
