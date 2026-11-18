/* The NI USB-621x protocol, as decoded from USB captures of NI-DAQmx: message
 * framing, the AI, AO and DIO messages, and the model table. Plain C++17, no
 * EPICS and no libusb, so the tests build it natively. The Python reference
 * (python/ni62xx.py) carries the same tables; tests/test_cpp.py holds the two
 * to the same bytes.
 *
 * A message on bulk ep01:
 *     00 ch | len(2) | len-4(2) | kind | verb | 03 | subsys | index(2) | payload
 * A kind-1 message gets exactly one ep81 reply on the same channel; reply byte
 * 11 is the status (02 ok, 06 refused) and data follows from byte 12. A
 * kind-0 message announces a bulk transfer on a pipe and gets no reply. */
#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace niusb {

using Bytes = std::vector<uint8_t>;

struct Error : std::runtime_error {
    using std::runtime_error::runtime_error;
};
/* The transport failed (a USB error, the unit gone): the port reconnects. */
struct TransportError : Error {
    using Error::Error;
};
struct Timeout : TransportError {
    using TransportError::TransportError;
};

constexpr uint16_t VID = 0x3923;
constexpr uint16_t PID_BLANK = 0x7269;   // an FX2 with no firmware: every 621x at power-up
constexpr uint8_t EP_CMD = 0x01, EP_REPLY = 0x81, EP_FPGA = 0x06, EP_DATA = 0x88;
constexpr double TIMEBASE = 20e6;        // the AI sample clock is TIMEBASE / divisor
constexpr unsigned SELF_CAL_TABLE = 0x1E4; // EEPROM: 7 x (gain V/code, offset V), BE float32
constexpr unsigned EEPROM_SIZE = 1024;
constexpr unsigned DO_SHIFT = 4;         // the PFI word: P0 in bits 0-3, P1 in bits 4-7

/* A model after its firmware load: the product ID it renumerates as. */
struct Model {
    uint16_t pid;
    const char *name;
    int nAi;                       // single-ended channels
    std::vector<unsigned> aoCal;   // EEPROM address of each AO's (codes/V, offset codes)
    int nDi, nDo;                  // P0 inputs, P1 outputs
};
const std::vector<Model> &models();
const Model *modelByPid(uint16_t pid);

enum class Mode { DIFF = 1, NRSE = 2, RSE = 3 };
/* The 621x input ranges, +-V, in the order of the records' menu. */
const std::vector<double> &ranges();
int gainCode(double range);   // throws Error on a range the 621x lacks
int calSlot(double range);    // the range's pair in the EEPROM self-cal table

Bytes msg(uint16_t ch, uint8_t verb, uint8_t subsys, uint16_t index = 0, const Bytes &payload = {}, uint8_t kind = 1);
Bytes announce(uint16_t ch, uint16_t pipe, uint32_t nbytes);
Bytes fromHex(const std::string &hex);
std::string toHex(const Bytes &b);

/* The AI channel list: u16 byte count, one word per channel (bit 14 on the
 * last; gain << 9, mode << 6, channel), padded to 4 bytes. */
Bytes aiConfig(const std::vector<int> &chans, double range, Mode mode);
/* Everything before the start: divisor 0 is on demand, else a finite timed
 * acquisition of n samples per channel. */
std::vector<Bytes> aiSetup(const std::vector<int> &chans, double range, Mode mode, uint32_t divisor = 0, uint32_t n = 0);
const std::vector<Bytes> &aiSample();               // one on-demand sample: announce 2 bytes, convert
Bytes aiStart();                                     // a timed acquisition's start
const std::vector<Bytes> &aiTeardown(bool timed);

/* Sent after both FPGAs report done: the two before NI-DAQmx's EEPROM read,
 * then the rest (DAQmx 20's order). */
const std::vector<Bytes> &postFpgaHead();
const std::vector<Bytes> &postFpgaTail();
/* Before the FPGAs: the three status queries NI-DAQmx starts with (no check). */
const std::vector<Bytes> &bringupHead();

Bytes eepromOpen();
Bytes eepromRead(uint32_t offset, uint32_t n);
constexpr uint32_t EEPROM_CHUNK = 0x2B;

std::vector<Bytes> aoWrite(uint16_t chan, int16_t code);
Bytes pfiRead();
std::vector<Bytes> doWrite(uint8_t value);

/* Big-endian helpers. */
uint32_t be32(const uint8_t *p);
float beFloat(const uint8_t *p);

} // namespace niusb
