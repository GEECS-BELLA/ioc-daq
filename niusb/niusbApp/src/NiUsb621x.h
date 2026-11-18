/* One NI USB-621x unit: finding it, loading NI's firmware into a blank one,
 * bringing up its FPGAs, reading its calibration, and the I/O. Plain C++17
 * over a Bus/Transport (libusb, the fake, or the tests' replay). Not thread
 * safe: the asyn port serializes every call. */
#pragma once

#include <array>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "NiUsbFirmware.h"
#include "NiUsbTransport.h"

namespace niusb {

class Device {
public:
    /* serial: the unit's USB serial (e.g. 016CD81D), or "" for the first.
     * fw: NI's firmware, needed only when a blank unit has to be loaded. */
    Device(Bus &bus, std::string serial, const Firmware *fw);

    /* Find the unit and make it ready: an already loaded unit is opened
     * as it is; blank units are loaded one by one until the one with the
     * serial appears (the others stay loaded for their own IOCs). */
    void open();
    void close();
    bool isOpen() const { return t_ != nullptr; }

    const Model &model() const;
    const std::string &serial() const { return info_.serial; }
    const Bytes &eeprom() const { return eeprom_; }
    /* The self-cal pair (V per code, offset V) of a range. */
    std::pair<double, double> aiCal(double range) const;

    /* On-demand reads of one channel: n raw codes, and their mean in volts. */
    std::vector<int16_t> aiReadCodes(int chan, double range, Mode mode, int n);
    double aiRead(int chan, double range, Mode mode, int n = 1);
    /* A finite hardware-timed scan, n samples per channel at rate Hz (the
     * clock is TIMEBASE / round(TIMEBASE / rate)); the codes interleaved. */
    std::vector<int16_t> aiTimedCodes(const std::vector<int> &chans, double rate, uint32_t n, double range, Mode mode);
    double aiTimedRate(double rate) const;
    double scale(int16_t code, double range) const;

    /* Static AO, calibrated; returns the code written. */
    int16_t aoWrite(int chan, double volts);
    /* The PFI word: P0 (inputs) in bits 0-3, P1 (outputs, read back) in 4-7. */
    uint32_t pfiRead();
    void doWrite(uint8_t value);

    /* For the tests: adopt an open transport and a known EEPROM, no I/O. */
    void adopt(std::unique_ptr<Transport> t, const UnitInfo &u, const Bytes &eeprom);

    /* Low level, for the tests: one message, its reply (empty for kind 0). */
    Bytes send(const Bytes &m);
    Bytes check(const Bytes &m);

    double replyTimeout = 10.0;   // s

private:
    void attach(const UnitInfo &u, bool fresh);
    void loadFx2(const UnitInfo &u);
    UnitInfo waitLoaded(const UnitInfo &blank);
    void fx2Write(Transport &t, uint8_t req, uint16_t base, const Bytes &data);
    void fpga(uint8_t target, const Bytes &image);
    void bringup();
    void readEeprom();
    void setEeprom(const Bytes &eeprom);
    void teardown(bool timed);
    const Model &requireModel() const;

    Bus &bus_;
    std::string want_;
    const Firmware *fw_;
    std::unique_ptr<Transport> t_;
    UnitInfo info_;
    const Model *model_ = nullptr;
    Bytes eeprom_;
    std::array<std::pair<double, double>, 7> cal_{};
    std::vector<std::pair<double, double>> aoCal_;
};

} // namespace niusb
