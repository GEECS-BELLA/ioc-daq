/* One NI USB-6008: its protocol, as decoded from USB captures of NI-DAQmx
 * 20, and the unit (finding it, its calibration memory, the I/O). Plain
 * C++17 over the same Bus/Transport as the 621x (libusb, the fake, the tests'
 * replay). The Python reference (python/ni6008.py) carries the same
 * messages; tests/test_cpp.py holds the two to NI-DAQmx's bytes. Not thread
 * safe: the asyn port serializes every call.
 *
 * The 6008 needs no firmware: it enumerates ready as 3923:717A. Its framing is
 * the 621x's with family byte 02:
 *     00 ch | len(2) | len-4(2) | kind | verb | 02 | subsys | index(2) | payload
 * A kind-1 message gets one ep81 reply on the same channel (byte 11 the
 * status, 02 ok; data from byte 12); a kind-0 message asks for AI samples,
 * which arrive on ep82 as big-endian 16-bit words (the 12-bit code in the top
 * 12 bits). Subsystems: 00 AI timing, 02 AI channels, 03 AI data, 04
 * calibration memory, 10 DIO, 21 AO.
 *
 * Opening the unit reads its calibration and nothing else: no AO write, no
 * DIO direction, so an IOC restart leaves the outputs as they are. */
#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "NiUsbTransport.h"

namespace niusb {
namespace n6008 {

constexpr uint16_t PID = 0x717A;
constexpr const char *MODEL = "USB-6008";
constexpr uint8_t EP_SAMPLES = 0x82;
constexpr uint8_t SUB_AI_TIMING = 0x00, SUB_AI_CHAN = 0x02, SUB_AI_DATA = 0x03, SUB_CAL = 0x04, SUB_DIO = 0x10,
                  SUB_AO = 0x21;
constexpr unsigned CAL_SIZE = 512, CAL_CHUNK = 0x20;
constexpr unsigned CAL_DIFF = 0x10, CAL_RSE = 0x110, CAL_AO = 0x150;
constexpr int N_RSE = 8, N_DIFF = 4, N_AO = 2, N_DI = 8, N_DO = 4;
constexpr double RSE_RANGE = 10.0;      // the only RSE range
constexpr double AO_MAX = 5.0;          // AO 0..5 V
constexpr double CLOCK = 24e6;          // the AI scan clock is CLOCK / divisor
constexpr uint32_t MAX_DIVISOR = 0xFFFF;// larger divisors are not captured (rates below 367 Hz)
constexpr double MAX_AI_RATE = 10e3;    // aggregate samples/s
constexpr int POLL_MS = 50;             // NI-DAQmx's poll during a timed read

/* The DIFF ranges, +-V, in the calibration memory's order (also the records' menu). */
const std::vector<double> &diffRanges();
int diffRangeIndex(double range);       // throws Error on a range the 6008 lacks

enum class Term { RSE, DIFF };

Bytes msg(uint8_t verb, uint8_t subsys, const Bytes &payload = {}, uint16_t index = 0, uint8_t kind = 1,
          uint16_t ch = 1);
Bytes calRead(uint32_t addr, uint16_t n = CAL_CHUNK);
/* Channel list, modes and range (one range for every channel). */
std::vector<Bytes> aiConfig(const std::vector<int> &chans, double range, Term mode);
/* Everything of an on-demand read of n samples, up to and including the
 * kind-0 request (the last message). */
std::vector<Bytes> aiOnDemand(int chan, double range, Term mode, uint32_t n);
/* Everything of a timed read up to and including its start (0a 02 00). */
std::vector<Bytes> aiTimed(const std::vector<int> &chans, double range, Term mode, uint32_t divisor, uint32_t n);
const std::vector<Bytes> &aiPoll();     // 13 02 03, 10 02 03: NI-DAQmx's poll while a timed read runs
const std::vector<Bytes> &aiTeardown();
Bytes dioRead(uint8_t port);
Bytes dioWrite(uint8_t port, uint8_t value);
Bytes dioDirection(uint8_t p0, uint8_t p1);   // 1 = output
Bytes aoWrite(uint16_t chan, int16_t code);

/* The calibration memory, parsed: (volts at the low end, volts at the high
 * end) of each range. */
struct Calibration {
    std::array<std::array<std::pair<double, double>, N_DIFF>, 8> diff{};   // [range index][chan]
    std::array<std::pair<double, double>, N_RSE> rse{};
    std::array<std::pair<double, double>, N_AO> ao{};
    static Calibration parse(const Bytes &mem);
};

} // namespace n6008

class Device6008 {
public:
    /* serial: the unit's USB serial (e.g. 019CDC74), or "" for the first. */
    Device6008(Bus &bus, std::string serial);

    void open();
    void close();
    bool isOpen() const { return t_ != nullptr; }
    const std::string &serial() const { return info_.serial; }
    const Bytes &calMemory() const { return mem_; }
    const n6008::Calibration &calibration() const { return cal_; }

    /* On-demand reads of one channel in one request: n words, and their mean in volts. */
    std::vector<uint16_t> aiReadWords(int chan, double range, n6008::Term mode, uint32_t n);
    double aiRead(int chan, double range, n6008::Term mode, uint32_t n = 1);
    /* A finite hardware-timed scan, n samples per channel at rate scans/s on
     * the divided 24 MHz clock: the words interleaved. */
    std::vector<uint16_t> aiTimedWords(const std::vector<int> &chans, double rate, uint32_t n, double range,
                                       n6008::Term mode);
    /* The divisor for a rate; throws above 10 kS/s aggregate or beyond 16 bits. */
    static uint32_t timedDivisor(size_t nChans, double rate);
    double scale(uint16_t word, double range, n6008::Term mode, int chan) const;

    /* Static AO, clamped to 0..5 V and calibrated; returns the code written. */
    int16_t aoCode(int chan, double volts) const;
    int16_t aoWrite(int chan, double volts);
    /* A port's lines: P0 8, P1 4 (P1's outputs read back). */
    uint8_t diRead(uint8_t port);
    /* P1 as outputs: the lines in mask take value's, the others keep the
     * state read just before; returns the state written. The first write
     * after an open latches P1's state and makes P1 outputs, as NI-DAQmx. */
    uint8_t doWrite(uint8_t value, uint8_t mask = 0xF);

    /* For the tests: adopt an open transport and a known calibration memory, no I/O. */
    void adopt(std::unique_ptr<Transport> t, const UnitInfo &u, const Bytes &mem);

    Bytes send(const Bytes &m);
    Bytes check(const Bytes &m);

    double replyTimeout = 10.0;   // s

private:
    void attach(const UnitInfo &u);
    Bytes readCal();
    void setCal(const Bytes &mem);
    void teardown();
    Bytes samples(size_t want, double seconds, bool poll);

    Bus &bus_;
    std::string want_;
    std::unique_ptr<Transport> t_;
    UnitInfo info_;
    Bytes mem_;
    n6008::Calibration cal_;
    bool dirSet_ = false;
};

} // namespace niusb
