/* A device-level fake of the USB-6008 (geecs-epics decision 13): a Bus whose
 * units answer the protocol as a 6008 did under NI-DAQmx. Each unit's
 * calibration memory is the one NI-DAQmx read from the captured unit at
 * plug-in; AO0 loops back to AI0 and AO1 to AI1, P1.0-3 to P0.0-3 (P0.4-7
 * and an undriven P1 read high, as the 6008's pull-ups do); the other AI
 * channels read 0.1 V x channel. A unit can be unplugged and plugged back.
 * The IOC's simulate mode runs on it, as do the tests. */
#pragma once

#include <deque>
#include <memory>
#include <mutex>

#include "NiUsb6008.h"

namespace niusb {

struct Fake6008Unit {
    explicit Fake6008Unit(std::string serial);

    std::string serial;
    bool present = true;
    bool stallTimed = false;      // a timed scan starts but its samples never come
    Bytes mem;                    // the calibration memory
    n6008::Calibration cal;
    int16_t ao[2] = {0, 0};
    bool aoWritten[2] = {false, false};
    uint8_t p1Latch = 0, p0Dir = 0, p1Dir = 0;
    std::vector<int> aiChans;
    std::vector<uint8_t> aiModes;
    int32_t rangeMv = 10000;
    bool timed = false;
    uint32_t count = 0;
    std::deque<Bytes> replies;
    Bytes data;
    std::vector<Bytes> received;  // every ep01 message, for the tests

    double input(int chan) const; // the voltage on an AI terminal
    uint16_t word(int chan) const;// the sample word for a list entry's channel
    void command(const Bytes &m);
};

class Fake6008Bus : public Bus {
public:
    std::vector<std::shared_ptr<Fake6008Unit>> units;
    std::mutex lock;

    std::shared_ptr<Fake6008Unit> add(const std::string &serial);
    std::vector<UnitInfo> list() override;
    std::unique_ptr<Transport> open(const UnitInfo &unit) override;
    void sleep(double) override {}
};

/* The calibration memory NI-DAQmx read from the captured unit (512 bytes). */
const Bytes &fake6008Memory();

} // namespace niusb
