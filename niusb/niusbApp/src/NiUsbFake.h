/* A device-level fake of the 621x (geecs-epics decision 13): a Bus whose
 * units answer the protocol as a USB-6211 did on the bench. A blank unit takes
 * the FX2 load and comes back loaded under its model's product ID; the FPGAs
 * report done once streamed; the EEPROM serves a nominal calibration; AO0 and
 * AO1 loop back to AI1 and AI2, P1.0 to P0.0, and P1 reads back in the PFI
 * word; other channels read 0.1 V x channel. The IOC's simulate mode runs on
 * it, as do the tests. */
#pragma once

#include <deque>
#include <memory>
#include <mutex>

#include "NiUsbTransport.h"

namespace niusb {

struct FakeUnit {
    FakeUnit(uint16_t pid, std::string serial, bool loaded);

    uint16_t pid;
    std::string serial;
    bool loaded, fpga0 = false, fpga1 = false;
    int renumerations = 0;
    // FX2 load
    uint8_t cpucs = 0;
    bool banksWritten = false, mainWritten = false;
    std::vector<std::pair<uint32_t, Bytes>> fx2Writes;   // (req << 16 | addr, data)
    // state
    Bytes eeprom;
    int16_t ao[2] = {1, 1};
    uint8_t doValue = 0;
    std::vector<int> aiChans;
    int aiGain = 0;
    uint32_t timedMode = 0, timedN = 0;
    uint8_t fpgaArmed = 0;
    size_t fpgaBytes = 0;
    std::deque<Bytes> replies;
    Bytes data;
    std::vector<Bytes> received;   // every ep01 message, for the tests

    double input(int chan) const;  // the voltage on an AI
    void command(const Bytes &m);
};

class FakeBus : public Bus {
public:
    std::vector<std::shared_ptr<FakeUnit>> units;
    std::mutex lock;

    /* A 6211 (or 6210) ready to use, or blank (needs the firmware). */
    std::shared_ptr<FakeUnit> add(uint16_t pid, const std::string &serial, bool loaded);
    std::vector<UnitInfo> list() override;
    std::unique_ptr<Transport> open(const UnitInfo &unit) override;
    void sleep(double) override {}
};

/* A nominal EEPROM for a model: its product ID first, the self-cal table,
 * the AO calibration. */
Bytes fakeEeprom(uint16_t pid);

} // namespace niusb
