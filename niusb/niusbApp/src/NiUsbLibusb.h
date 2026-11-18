/* The 621x bus on libusb-1.0. */
#pragma once

#include "NiUsbTransport.h"

struct libusb_context;

namespace niusb {

class LibusbBus : public Bus {
public:
    LibusbBus();
    ~LibusbBus() override;
    std::vector<UnitInfo> list() override;
    std::unique_ptr<Transport> open(const UnitInfo &unit) override;
    void sleep(double seconds) override;

private:
    libusb_context *ctx_ = nullptr;
};

} // namespace niusb
