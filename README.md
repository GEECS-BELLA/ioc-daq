# Generic IOC ioc-daq

Multifunction DAQ modules on plain EPICS records, for GEECS (geecs-epics
decision 14: a DAQ used for slow analog and digital I/O is a setpoint
device, so plain records; decision 11: one image per family). Built on
epics-containers' `epics-base` developer image with the template's iocStats,
pvlogging and autosave, plus:

| Module | Version | Source | Reuse (decision 10) |
|---|---|---|---|
| asyn | R4-46 | upstream ibek-support recipe | tier 1, dependency |
| busy | R1-7-4 | upstream ibek-support recipe (record type used by measComp's databases) | tier 1, dependency |
| calc | R3-7-5 | upstream ibek-support recipe (calcout records in measComp's databases) | tier 1, dependency |
| measComp | master 9c8e01e (2026-07-13) | [epics-modules/measComp](https://github.com/epics-modules/measComp) (APS, Mark Rivers), our recipe `ibek-support-geecs/measComp/` | tier 1, dependency; no LICENSE file (below) |
| uldaq | v1.2.2 | [mccdaq/uldaq](https://github.com/mccdaq/uldaq), MCC's UL for Linux, MIT, built from source by the measComp recipe | vendor, open source |

No epics-containers image or ibek-support recipe carries measComp (checked
2026-10-30, GitHub code search included), hence this one. First device: an
MCC USB-1808X (geecs-epics `services/undulator/daq/daqtest1`).

## measComp

- **Pin.** Master at 9c8e01e rather than a tag: R4-4 is in the release notes
  but untagged, and the last tag, R4-3 (2024-02), lacks R4-4's poller fix
  that forces callbacks for averaged analog inputs, which a slowly varying
  readback depends on.
- **What is built.** `drvMultiFunction` (the multifunction modules: USB-1808,
  USB-1808X, USB-1608G, USB-2408, USB-31xx, E-1608, E-DIO24, E-TC, TC-32,
  USB-TEMP, ...) and `measCompDiscover`. The recipe comments out the USB-CTR
  scaler/MCS driver and its SNL program (they need scaler, mca and the
  sequencer), measComp's own IOC applications and test programs, and the
  demo and iocBoot directories. Nothing in the driver is patched.
  `libmeasComp.so` links uldaq and libusb itself, with an rpath to
  `/usr/local/lib`, so the IOC links it like any other support library.
- **USB-1808X.** The driver treats the 1808X as the USB-1808 board family
  (`USB_1808X` = 318). Its records are upstream's `USB1808.substitutions`.
- **Licence.** measComp ships no LICENSE file anywhere in the repository.
  geecs-epics took it as a tier-1 dependency on the same basis as
  motorAerotech: the `epics-modules` org, synApps code by APS (the
  maintainer's decision, 2026-10-30). The module is fetched at build time,
  not vendored.

## The entity

`measComp.MultiFunction` (`ibek-support-geecs/measComp/measComp.ibek.support.yaml`):
`MultiFunctionConfig(name, unique_id, wdig_points, wgen_points)` and
`dbLoadTemplate` of upstream's `<model>.substitutions` with `P`, `PORT`,
`WDIG_POINTS`, `WGEN_POINTS`; after iocInit, upstream's two `dbpf ...Dwell.PROC`
lines. `unique_id` is the USB serial number (`lsusb -v`'s `iSerial`).
`wdig_points` and `wgen_points` default to 16384, not upstream's 1048576:
the arrays are allocated whether the digitizer runs or not, and GEECS uses
these modules for slow I/O.

## What the driver sends at start

From measComp 9c8e01e's `drvMultiFunction.cpp` and its templates:

1. `MultiFunctionConfig` finds the module by serial number, connects, and
   reads its identity and capabilities. It starts the poller, which reads
   the analog inputs, digital inputs and counters every `PollSleepMS` (50 ms).
2. At iocInit, the records with `PINI YES` write. In upstream's templates
   those are the analog output values (`ao`, `$(PINI=YES)`), the binary
   outputs (`bo`, `$(PINI=YES)`), the digital direction bits
   (`measCompBinaryDir`, VAL 0 = input), the analog-input modes and ranges,
   the analog-output ranges and sync bits, the trigger mode, the pulse
   generators' settings (not their run state) and `PollSleepMS`. With no
   autosave, every analog output is driven to 0 V and every DIO bit is set
   to input at each start.
3. `outputs_pini: "NO"` on the entity passes `PINI=NO` to
   `dbLoadTemplate`. That leaves the analog and binary output values alone
   at start. The directions and ranges are still written, and the records
   read 0 until written, because uldaq has no readback of an output. A
   production instance whose outputs drive something chooses this
   deliberately (or keeps autosave).

### Testing without hardware

None yet: measComp has no simulation mode, and decision 13's fake for this
family is open in geecs-epics (`docs/device-catalogue.md`).

## Build

As ioc-plc. No closed vendor code is inside, so the image is published: CI
builds on every push and, on a git tag (CalVer `YYYY.M.N`, never re-pushed),
pushes `ghcr.io/geecs-bella/ioc-daq-runtime:<tag>` and `-developer:<tag>`.
Both packages are made public once, by hand, after the first tag creates
them. A pre-release tag (`2026.10.1b1`) publishes a branch for bench work.

```
DOCKER_DEFAULT_PLATFORM=linux/amd64 TARGET=runtime TAG=ioc-daq:local ./build  # amd64, emulated on Apple silicon
```

## Licence

GEECS's own work here (the Dockerfile changes, the ibek support files under
`ibek-support-geecs/`, the docs) is under `LICENSE`, the GEECS licence:
BSD-style, Copyright The Regents of the University of California, through
Lawrence Berkeley National Laboratory, as in GEECS-Plugins. The files
generated from epics-containers' ioc-template keep its Apache-2.0 licence,
`LICENSE-ioc-template`. The EPICS modules and uldaq the image builds
(fetched at build time, not vendored) carry their own licences, listed in
the table above.

## Template Info
This repository was generated by
[ioc-template](https://github.com/epics-containers/ioc-template) 4.8.0
(`copier update -a --trust .` to update), by way of GEECS-BELLA/ioc-plc's
copy of it. The devcontainer and `./build` work as in every
epics-containers generic IOC.
