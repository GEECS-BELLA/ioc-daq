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
    void refresh();
    libusb_context *ctx_ = nullptr;
    int live_ = 0;  // transports open on ctx_; it is renewed only at 0
};

} // namespace niusb
