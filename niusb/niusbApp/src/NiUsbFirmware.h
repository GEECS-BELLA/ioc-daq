/* NI's firmware for the 621x, read at run time from a folder the host
 * provides (geecs-epics decision 23): five files of an NI-DAQmx install
 * (NI-DAQ\fpgaFiles\miousb\), never stored in this repository or its image.
 * Each is checked against the SHA-256 of the DAQmx 20 file it was taken from;
 * a file that differs is refused, by name. */
#pragma once

#include <string>

#include "NiUsbProtocol.h"

namespace niusb {

struct FirmwareFile {
    const char *name;     // as NI-DAQmx installs it
    const char *sha256;   // NI-DAQmx 20
};
/* usb621x.cfg, usb621x0.cfg, usb621x1.cfg, SlicFpga.cfg, STC3Lite.cfg. */
const std::vector<FirmwareFile> &firmwareFiles();

struct Firmware {
    Bytes loader;    // FX2 RAM at 0, under CPUCS hold (usb621x.cfg)
    Bytes main16k;   // FX2 RAM at 0 after the banks (the first 16 KB of usb621x0.cfg and usb621x1.cfg)
    Bytes bank0;     // request 0xA4 at 0x4000 (usb621x0.cfg after 16 KB)
    Bytes bank1;     // request 0xA5 at 0x4000 (usb621x1.cfg after 16 KB)
    Bytes fpga0;     // SlicFpga.cfg
    Bytes fpga1;     // STC3Lite.cfg

    /* Read and check the five files in dir; throws Error naming the file. */
    static Firmware load(const std::string &dir, bool checkSums = true);
};

std::string sha256Hex(const Bytes &data);

} // namespace niusb
