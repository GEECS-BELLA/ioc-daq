/* What the device code needs from USB, so the same code runs on libusb, on
 * the fake (NiUsbFake) and on the tests' replay of a NI-DAQmx capture. */
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "NiUsbProtocol.h"

namespace niusb {

/* One opened unit. Errors are TransportError (Timeout on a timeout). */
class Transport {
public:
    virtual ~Transport() = default;
    /* Vendor control requests on the default pipe (bmRequestType 0x40 / 0xC0). */
    virtual void ctrlOut(uint8_t req, uint16_t value, uint16_t index, const Bytes &data, int timeoutMs) = 0;
    virtual Bytes ctrlIn(uint8_t req, uint16_t value, uint16_t index, uint16_t length, int timeoutMs) = 0;
    virtual void write(uint8_t ep, const Bytes &data, int timeoutMs) = 0;
    /* One transfer of at most maxLen bytes. */
    virtual Bytes read(uint8_t ep, size_t maxLen, int timeoutMs) = 0;
};

/* A unit on the bus as enumerated: blank (PID_BLANK, no serial) or loaded. */
struct UnitInfo {
    uint16_t pid = 0;
    std::string serial;
    int bus = 0;
    std::vector<int> ports;   // the port path, which survives a renumeration
    bool samePlace(const UnitInfo &o) const { return bus == o.bus && ports == o.ports; }
};

class Bus {
public:
    virtual ~Bus() = default;
    virtual std::vector<UnitInfo> list() = 0;
    /* Open a unit. A loaded one gets its interface claimed (and a
     * configuration set only if it has none: a second SET_CONFIGURATION
     * wedges this firmware); a blank one needs only its default pipe. */
    virtual std::unique_ptr<Transport> open(const UnitInfo &unit) = 0;
    virtual void sleep(double seconds) = 0;
};

} // namespace niusb
