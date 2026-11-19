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
| niusb | this repository (`niusb/`, recipe `ibek-support-geecs/niusb/`) | ours: NI USB-621x and USB-6008 over libusb-1.0, from our decoding of the USB protocol; no NI code | written |

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

measComp: none yet (measComp has no simulation mode). niusb: the entity's
`simulate: true` runs the unit on the driver's fake ("NI USB-621x" and "NI
USB-6008" below), and the CI instance (`tests/config/ioc.yaml`) checks its
loopbacks.

## NI USB-621x

The USB-6211 and USB-6210 (GEECS's `DaqPad_NI6009` devices) on our own
driver, `niusb`: no NI-DAQmx, no NI library. The protocol was decoded from
USB captures of NI-DAQmx 20 driving a USB-6211; the Python reference
(`python/ni62xx.py`) ran a USB-6211 and a USB-6210 from Linux (on-demand
and timed AI against loopbacks, AO within a millivolt, DI/DO), and the C++
driver (`niusb/niusbApp/src/`) reproduces its requests byte for byte.

- **Power-up.** A 621x enumerates blank (`3923:7269`, an FX2 with no
  firmware) and takes, over USB, a loader, its 8051 firmware and two FPGA
  bitstreams, after which it comes back as its model (`3923:7270` the
  6211, `3923:726F` the 6210). The driver does this whenever it finds a
  blank unit, then reads the unit's EEPROM: the self-calibration of each
  AI range and, on the 6211, of AO0 and AO1 (both measured against a
  loopback on the bench).
- **NI's firmware is not here** (geecs-epics decision 23). It is five files
  of an NI-DAQmx install (`NI-DAQ\fpgaFiles\miousb\`): `usb621x.cfg`,
  `usb621x0.cfg`, `usb621x1.cfg`, `SlicFpga.cfg`, `STC3Lite.cfg`, copied
  from a lab PC to the host and mounted read-only at `fw_dir`
  (`/opt/ni-firmware`). Each is checked against the SHA-256 of the NI-DAQmx
  20 file (`NiUsbFirmware.cpp`); a differing file is refused by name, in
  the `Firmware` record. A unit already loaded (the IOC restarted, the unit
  powered throughout) needs no firmware.
- **Several units.** `serial` picks one (lsusb's `iSerial` once loaded). A
  blank unit has no serial: the driver loads blank units one by one until
  its own appears; the others stay loaded, ready for their own IOCs. The
  loading runs under an exclusive `flock` on `fw_dir`, which every NI IOC
  on the host mounts from the same host folder (a lock on another copy
  locks nothing), and the bus is listed again once the lock is held, so
  two IOCs starting together never load one unit at once (on the fake,
  and on the bench: two IOCs on one blank unit, one loaded it and the
  other was refused `LIBUSB_ERROR_BUSY` and kept retrying). A lock that
  cannot be taken is logged as a warning and the load goes ahead.
- **Unplugged or power-cycled.** The IOC marks the unit disconnected at the
  first failed transfer, retries every 5 s, and reloads it when it comes
  back blank (22 s from the pull to serving on the bench). libusb's device
  list is fed by udev's hotplug events, which never reach a container with
  no udevd, so the driver makes a new libusb context for each scan of the
  bus while no unit is open.
- **Records** (measComp's record names only: not its drvInfo strings, its
  full record sets or its screens; `Db/USB6211.substitutions`):
  `Ai1`-`Ai16` (on demand, `SCAN` 1 s; `Ai<n>Range`, `Ai<n>Mode` RSE/NRSE/
  DIFF, `Ai<n>Average`), `Ao1`-`Ao2` (6211; no readback exists, so the
  record reads 0 until written; `PINI` NO: an IOC start leaves the outputs
  as they are), `Bi1`-`Bi4` (P0.0-3), `Bo1`-`Bo4` (P1.0-3, `PINI` NO) with
  `Bo<n>_RBV` (the unit's readback), the `WaveDig` finite timed scan
  (`WaveDigNumPoints`, `FirstChan`, `NumChans`, `Dwell` on the 20 MHz
  clock with `DwellActual`, `Range`, `Mode`, `Run`; arrays `WaveDig1`-`16`
  and `WaveDigTimeWF`), and `Model`, `SerialNumber`, `Connected`,
  `LastError`, `Firmware`. After each open the driver takes P1's state from
  the unit, so a write to one `Bo` keeps the other lines as they are; when
  the IOC saw its unit blank during the open (it lost power; this IOC or
  another loaded it) `Ao*` read 0, as the unit does. If another IOC loaded
  it before this one listed the bus, `Ao*` keep their last setpoint.
- **WaveDig limits.** The 621x samples 250 kS/s in all: `NumChans` /
  `Dwell` above that is refused. A scan holds the port's thread for its
  whole length, the other records waiting, so a scan longer than 30 s is
  refused. `WaveDigRun` returns to Done either way; after a scan that did
  not run, `WaveDigOK` is Failed (MAJOR), the arrays are unchanged and the
  reason is in `LastError`.
- **Reconnects.** A USB error closes the unit; the next request reopens
  it, at most every 5 s, loading the firmware again if the unit came back
  blank. Records read INVALID meanwhile.
- **Not done:** continuous (streaming) AI, hardware triggers and PFI
  routing, counters, timed AO, the other 621x models (6212, 6215, 6216,
  6218: a line in the model table once a unit is in hand). The USB-6008 is
  below.
- **Tests.** `tests/fixtures/daqmx-621x.json` holds NI-DAQmx's traffic for
  the power-up and each kind of request, cut from the captures by
  `tools/cut_fixture.py` with NI's bytes replaced by their length and
  SHA-256; `tests/test_python.py` and `tests/test_cpp.py` (native build,
  `tests/cpp/`) replay it against both drivers (the `Device` layer): the
  ep01 messages byte for byte, the FX2 requests and the FPGA streams by
  address and length (their bytes are NI's). One difference is allowed:
  NI-DAQmx's first DO write skips an unchanged 0, the drivers always write
  it (its reply is supplied in the replay). The C++ driver also runs
  against the fake (open, reopen, failures, loopbacks, limits). The asyn
  layer is exercised by CI's simulated 6211 and 6210 (`run-tests.sh`):
  Ao2 → Ai3, Bo1 → Bi1, a WaveDig scan, the models. `pytest tests/` needs
  only a C++17 compiler.

## NI USB-6008

The USB-6008 on the same module, `niusb` (`NiUsb6008.*`, `drvNiUsb6008.cpp`,
the entity `niusb.USB6008`), with the 621x's record names so GEECS sees one
DAQ family. The protocol was decoded from USB captures of NI-DAQmx 20 driving
a USB-6008 (plug-in, reads, writes against loopbacks AO0→AI0, AO1→AI1,
P1.0→P0.0); the Python reference is `python/ni6008.py`, and the C++ driver
makes its requests byte for byte. Phases A and B passed on a USB-6008 from
Linux (2026-11-18), with the Python reference and with the IOC over CA: AO
codes identical to DAQmx's, AO→AI within 7.6 mV over 0-4.5 V, DO patterns
exact, timed AI up to 10 kS/s × 2000.

- **Protocol.** No firmware and no FX2: the unit enumerates ready as
  `3923:717A` (full speed, interface 0, bulk ep01 out, ep81 and ep82 in). The
  621x's framing with family byte 02: a kind-1 message gets one ep81 reply
  (byte 11 the status), a kind-0 message asks for AI samples, which come on
  ep82 as big-endian 16-bit words, the 12-bit code in the top 12 bits.
  Subsystems: 00 AI timing, 02 AI channels, 03 AI data, 04 calibration
  memory, 10 DIO, 21 AO. An on-demand read is one request for n samples; a
  timed read sets a divisor of the 24 MHz clock and a count, starts with
  `0a 02 00`, and NI-DAQmx polls (`13 02 03`, `10 02 03`) every 50 ms until
  the samples are in, which the drivers repeat.
- **Calibration.** At each open the driver reads the unit's 512-byte
  calibration memory (32 bytes at a time) and nothing else: no AO write, no
  DIO direction, so an IOC restart leaves the outputs as they are (NI-DAQmx
  zeroes both AOs and resets the DIO directions at plug-in; the driver does
  not). The memory holds, per range, the volts at the low and the high end:
  DIFF on eight ranges × AI0-3 from 0x10, RSE ±10 V × AI0-7 from 0x110,
  AO0 and AO1 at 0x150 and 0x158; the unit's serial is at 0x1FC.
- **Scaling, against NI-DAQmx's own volts** (`tests/test_python.py`):
  DIFF `lo + (word + 0.5)·(hi − lo)/65536` matches DAQmx within 0.5 µV (9
  values on ±20, ±5 and ±1 V; AI0 only). RSE `hi + (int16(word) + 0.5)·
  (hi − lo)/32768` is within 3.5 mV of DAQmx (56 values on AI0, AI1, AI7,
  0 to 4.5 V): DAQmx's RSE line is exactly linear in the word but its ends
  are not the memory's pair; the formula behind it is not recovered (a third
  of an RSE step, 10 mV). AO: `round((V − lo)/(hi − lo)·4095)`, V clamped to
  0..5 V, equals the code DAQmx sent in all 10 writes, AO1's 0 V as −5.
- **DO.** A write to P1 reads P1 first and keeps the lines outside its mask
  as they read; the first write after an open latches P1's state and makes
  P1.0-3 outputs (P0 stays input), as NI-DAQmx did. An undriven line reads
  high (the 6008's pull-ups), so after a power-up `Bo*_RBV` read 1 and the
  first masked write drives the other lines high, where they already read.
- **Records** (`Db/USB6008.substitutions`): `Ai1`-`Ai8` (on demand, `SCAN`
  1 s; `Ai<n>Range` ±20, 10, 5, 4, 2.5, 2, 1.25, 1 V, default ±10;
  `Ai<n>Mode` RSE or DIFF; `Ai<n>Average`). RSE is ±10 V only; DIFF is
  AI0-3, each against AI n+4; anything else is refused into `LastError`.
  In RSE the Range record is ignored (RSE is always ±10 V), so switching a
  DIFF channel back to RSE needs no Range change.
  `Ao1`-`Ao2` (0..5 V, `PINI` NO, read 0 until written), `Bi1`-`Bi8`
  (P0.0-7), `Bo1`-`Bo4` (P1.0-3, `PINI` NO) with `Bo<n>_RBV`, the
  `WaveDig` scan (`Dwell` 0.1 to 2.73 ms on the 24 MHz clock, 10 kS/s
  aggregate, `WaveDig1`-`8`), and `Model`, `SerialNumber`, `Connected`,
  `LastError` (and `Firmware`, which says none is needed).
- **Reconnects** as the 621x: a USB error closes the unit, the next request
  reopens it at most every 5 s, and the libusb bus takes a new context for
  each scan while no unit is open.
- **Not done:** continuous AI, triggers, the counter, timed AO, P0 as
  output, timed AI below 367 Hz (a clock divisor beyond 16 bits: the
  captures show 16 bits in a 32-bit field, so the drivers refuse rather
  than guess), several ranges in one scan, and the USB-6009 (likely the
  same protocol at 14 bits; no unit in hand).
- **Proven on the bench too** (2026-11-18): a replug while the IOC ran
  (serving again 11 s after the pull), a `docker kill` during a timed scan
  followed by a restart with no replug (connected in 5 s, AI and a scan
  fine), and a 7.5 h soak over CA (8,255 cycles of AO → AI, DO patterns and
  scans; no IOC failure: the replug, and one AO readback 15 mV off, about 3
  counts of RSE ±10 V).
- **Tests.** `tests/fixtures/daqmx-6008.json` holds NI-DAQmx's traffic for
  each probe step and its calibration read at plug-in (the unit's data, not
  NI code), cut by `tools/cut_fixture_6008.py` with only the volts DAQmx
  returned from the probe logs; the counter step is dropped. In the replay
  (`tests/replay6008.py`) a reply is due only after as many messages as
  preceded it in the capture, so the open's drain and the timed read's
  polls come out as NI-DAQmx's. Both drivers send the fixture's messages
  for every step (P0 as output is not replayed), the C++ results equal the
  Python ones, and the C++ driver runs against the fake (loopbacks,
  refusals, unplug and replug, an open that writes nothing). CI's simulated
  6008 (`run-tests.sh`): Ao1 → Ai1, Bo1 → Bi1, a WaveDig scan, the model.

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
`ibek-support-geecs/`, the niusb driver, its Python reference and tests,
the docs) is under `LICENSE`, the GEECS licence:
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
