/* The NI bus on libusb-1.0: the 621x's units by default, or the product IDs
 * a predicate accepts (the 6008's). */
#pragma once

#include <functional>

#include "NiUsbTransport.h"

struct libusb_context;

namespace niusb {

class LibusbBus : public Bus {
public:
    /* pids: the product IDs (VID 3923) to list and open; none for the 621x's,
     * blank and loaded. */
    explicit LibusbBus(std::function<bool(uint16_t pid)> pids = {});
    ~LibusbBus() override;
    std::vector<UnitInfo> list() override;
    std::unique_ptr<Transport> open(const UnitInfo &unit) override;
    void sleep(double seconds) override;

private:
    void refresh();
    bool wanted(uint16_t vid, uint16_t pid) const;
    std::function<bool(uint16_t)> pids_;
    libusb_context *ctx_ = nullptr;
    int live_ = 0;  // transports open on ctx_; it is renewed only at 0
};

} // namespace niusb
