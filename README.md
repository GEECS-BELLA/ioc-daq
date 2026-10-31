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
| uldaq | v1.2.2 (f87594f; it reports itself as 1.2.0, the IOC's `ULVersion`) | [mccdaq/uldaq](https://github.com/mccdaq/uldaq), MCC's UL for Linux, MIT, built from source by the measComp recipe | vendor, open source |

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
  demo and iocBoot directories; `test_measCompDiscover` is kept. Nothing in
  the driver is patched.
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
`WDIG_POINTS`, `WGEN_POINTS`. After iocInit come upstream's `dbpf
WaveDigDwell.PROC` and `WaveGenUserDwell.PROC` lines, each only for the
models whose substitutions load those records. `model: USBCTR` cannot
work: the USB-CTR driver is not built. `test_measCompDiscover`
(`/epics/support/measComp/bin/linux-x86_64/`) lists the modules on the bus
with their serial numbers. `unique_id` is the USB serial number (`lsusb -v`'s `iSerial`).
`wdig_points` and `wgen_points` default to 16384, not upstream's 1048576:
the arrays are allocated whether the digitizer runs or not, and GEECS uses
these modules for slow I/O.

`substitutions` replaces upstream's file with one of the instance's own.
Use one for the USB-1808 and 1808X. Upstream's `USB1808.substitutions` (at
9c8e01e) gives Ai1-Ai8 rows copied from `USB1608G.substitutions`: RANGE
3/2/1/0 with EGU ±10/±5/±2/±1 V, which match neither model's enum (the
1608G's starts at ±10 V too). On the 1808 those indices mean 0-5 V, 0-10 V,
±5 V and ±10 V (`inputRangeUSB_1808`), so every input is scaled wrong; on the bench
Ai7 and Ai8 read 10× small. geecs-epics' `daqtest1.substitutions` is the
corrected copy. The `ai` records convert raw counts with EGUL/EGUF, so a
Range change at runtime needs matching EGU limits. That is a candidate to
report upstream.

Upstream's analog-output template also loads `AoNSyncMaster`/`SyncEnable`
with PINI. R4-4's synchronous DAC update supports only the USB-31xx, so on
the 1808X the start prints "Setting analog out master, err=24
Configuration not supported" for each channel. It is harmless and also a
candidate for upstream. These records read UDF (INVALID) until written, as
every `bo` with no VAL does (`Bo1`-`Bo4`, `AoNPulse`, `AoNSyncEnable`, ...);
that comes from the template, not from the error.

## What the driver sends at start

From measComp 9c8e01e's `drvMultiFunction.cpp` and its templates:

1. `MultiFunctionConfig` finds the module by serial number, connects, and
   reads its identity and capabilities. It starts the poller, which reads
   the analog inputs, digital inputs and counters every `PollSleepMS` (50 ms).
2. At iocInit, the records with `PINI YES` write. In upstream's templates
   those are:
   - the analog output values (`ao`, `$(PINI=YES)`) and the binary outputs
     (`bo`, `$(PINI=YES)`);
   - the digital direction bits (`measCompBinaryDir`, VAL 0 = input);
   - the counter and encoder resets (`CounterNReset`, `EncoderNReset`,
     VAL 1: `ulCLoad(..., 0)`), so **counts restart from 0 at each IOC
     start**;
   - the analog-input mode, ranges, types and rates;
   - the analog-output ranges and sync bits;
   - the trigger mode;
   - the pulse generators' settings (their `Run` is PINI too, but a stopped
     generator sends nothing);
   - the waveform digitizer's and generator's settings;
   - `PollSleepMS`.

   This image wires no autosave for measComp's records. So at each start
   every analog output is driven to 0 V and every DIO bit is set to input.
3. `outputs_pini: "NO"` on the entity passes `PINI=NO` to `dbLoadTemplate`.
   That leaves the analog output values alone at start: their records read 0
   until written, because uldaq has no readback of an output. Every DIO bit
   is still set to input by the direction records, which releases any
   digital output, and the ranges and counter resets are still written. A
   production instance whose outputs drive something needs its own
   substitutions (no PINI on `Bd*`, or the right VAL) and a decision on the
   resets.

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
