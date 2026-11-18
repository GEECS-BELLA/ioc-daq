#include "NiUsbLibusb.h"

#include <libusb-1.0/libusb.h>

#include <chrono>
#include <fstream>
#include <thread>

namespace niusb {

static std::string err(const char *what, int rc)
{
    return std::string(what) + ": " + libusb_error_name(rc);
}

static void fail(const char *what, int rc)
{
    if (rc == LIBUSB_ERROR_TIMEOUT)
        throw Timeout(err(what, rc));
    throw TransportError(err(what, rc));
}

namespace {

class LibusbTransport : public Transport {
public:
    LibusbTransport(libusb_device_handle *h, bool claimed, int &live) : h_(h), claimed_(claimed), live_(live)
    {
        live_++;
    }
    ~LibusbTransport() override
    {
        if (claimed_)
            libusb_release_interface(h_, 0);
        libusb_close(h_);
        live_--;
    }

    void ctrlOut(uint8_t req, uint16_t value, uint16_t index, const Bytes &data, int timeoutMs) override
    {
        Bytes d = data;
        int rc = libusb_control_transfer(h_, 0x40, req, value, index, d.data(), uint16_t(d.size()), unsigned(timeoutMs));
        if (rc < 0)
            fail("control out", rc);
        if (size_t(rc) != data.size())
            throw TransportError("control out: short transfer");
    }

    Bytes ctrlIn(uint8_t req, uint16_t value, uint16_t index, uint16_t length, int timeoutMs) override
    {
        Bytes d(length);
        int rc = libusb_control_transfer(h_, 0xC0, req, value, index, d.data(), length, unsigned(timeoutMs));
        if (rc < 0)
            fail("control in", rc);
        d.resize(size_t(rc));
        return d;
    }

    void write(uint8_t ep, const Bytes &data, int timeoutMs) override
    {
        Bytes d = data;
        int done = 0;
        int rc = libusb_bulk_transfer(h_, ep, d.data(), int(d.size()), &done, unsigned(timeoutMs));
        if (rc < 0)
            fail("bulk write", rc);
        if (size_t(done) != data.size())
            throw TransportError("bulk write: " + std::to_string(done) + " of " + std::to_string(data.size()) + " bytes");
    }

    Bytes read(uint8_t ep, size_t maxLen, int timeoutMs) override
    {
        Bytes d(maxLen);
        int done = 0;
        int rc = libusb_bulk_transfer(h_, ep, d.data(), int(maxLen), &done, unsigned(timeoutMs));
        if (rc < 0 && !(rc == LIBUSB_ERROR_TIMEOUT && done > 0))
            fail("bulk read", rc);
        d.resize(size_t(done));
        return d;
    }

private:
    libusb_device_handle *h_;
    bool claimed_;
    int &live_;
};

UnitInfo describe(libusb_device *dev, const libusb_device_descriptor &d)
{
    UnitInfo u;
    u.pid = d.idProduct;
    u.bus = libusb_get_bus_number(dev);
    uint8_t ports[8];
    int n = libusb_get_port_numbers(dev, ports, sizeof ports);
    for (int i = 0; i < n; i++)
        u.ports.push_back(ports[i]);
    // The serial as the kernel read it at enumeration (sysfs): a live string
    // request goes unanswered while a reply from an interrupted transaction
    // still waits on ep81 (seen after a killed process), so it comes second.
    std::string path = "/sys/bus/usb/devices/" + std::to_string(u.bus) + "-";
    for (size_t i = 0; i < u.ports.size(); i++)
        path += (i ? "." : "") + std::to_string(u.ports[i]);
    std::ifstream sys(path + "/serial");
    if (sys && std::getline(sys, u.serial) && !u.serial.empty())
        return u;
    u.serial.clear();
    if (d.iSerialNumber) {
        libusb_device_handle *h = nullptr;
        if (libusb_open(dev, &h) == 0) {
            unsigned char sn[64] = {0};
            if (libusb_get_string_descriptor_ascii(h, d.iSerialNumber, sn, sizeof sn - 1) > 0)
                u.serial = reinterpret_cast<char *>(sn);
            libusb_close(h);
        }
    }
    return u;
}

bool wanted(const libusb_device_descriptor &d)
{
    return d.idVendor == VID && (d.idProduct == PID_BLANK || modelByPid(d.idProduct));
}

} // namespace

LibusbBus::LibusbBus()
{
    int rc = libusb_init(&ctx_);
    if (rc)
        throw TransportError(err("libusb_init", rc));
}

LibusbBus::~LibusbBus()
{
    if (ctx_)
        libusb_exit(ctx_);
}

void LibusbBus::refresh()
{
    if (ctx_)
        libusb_exit(ctx_);
    ctx_ = nullptr;
    int rc = libusb_init(&ctx_);
    if (rc) {
        ctx_ = nullptr;
        throw TransportError(err("libusb_init", rc));
    }
}

std::vector<UnitInfo> LibusbBus::list()
{
    // libusb keeps its device list from hotplug events, which come from
    // udev; in a container without udev none arrive, so a unit unplugged,
    // replugged or renumerated after its firmware load is never seen. A new
    // context scans sysfs afresh: made whenever no unit is open (a list is
    // taken only while looking for one).
    if (live_ == 0 || !ctx_)
        refresh();
    libusb_device **devs = nullptr;
    ssize_t n = libusb_get_device_list(ctx_, &devs);
    if (n < 0)
        throw TransportError(err("device list", int(n)));
    std::vector<UnitInfo> out;
    for (ssize_t i = 0; i < n; i++) {
        libusb_device_descriptor d;
        if (libusb_get_device_descriptor(devs[i], &d) == 0 && wanted(d))
            out.push_back(describe(devs[i], d));
    }
    libusb_free_device_list(devs, 1);
    return out;
}

std::unique_ptr<Transport> LibusbBus::open(const UnitInfo &unit)
{
    if (!ctx_)
        refresh();
    libusb_device **devs = nullptr;
    ssize_t n = libusb_get_device_list(ctx_, &devs);
    if (n < 0)
        throw TransportError(err("device list", int(n)));
    libusb_device_handle *h = nullptr;
    int rc = LIBUSB_ERROR_NO_DEVICE;
    for (ssize_t i = 0; i < n && !h; i++) {
        libusb_device_descriptor d;
        if (libusb_get_device_descriptor(devs[i], &d) || !wanted(d) || d.idProduct != unit.pid)
            continue;
        UnitInfo here;
        here.bus = libusb_get_bus_number(devs[i]);
        uint8_t ports[8];
        int np = libusb_get_port_numbers(devs[i], ports, sizeof ports);
        for (int j = 0; j < np; j++)
            here.ports.push_back(ports[j]);
        if (here.samePlace(unit))
            rc = libusb_open(devs[i], &h);
    }
    libusb_free_device_list(devs, 1);
    if (!h)
        throw TransportError(err("open", rc));
    if (unit.pid == PID_BLANK)  // the FX2 load needs only the default pipe
        return std::make_unique<LibusbTransport>(h, false, live_);
    libusb_set_auto_detach_kernel_driver(h, 1);
    int cfg = 0;
    if (libusb_get_configuration(h, &cfg) == 0 && cfg == 0) {
        // a second SET_CONFIGURATION wedges this firmware: only when unconfigured
        rc = libusb_set_configuration(h, 1);
        if (rc) {
            libusb_close(h);
            throw TransportError(err("set configuration", rc));
        }
    }
    rc = libusb_claim_interface(h, 0);
    if (rc) {
        libusb_close(h);
        throw TransportError(err("claim", rc) + (rc == LIBUSB_ERROR_BUSY ? " (another process holds the unit)" : ""));
    }
    return std::make_unique<LibusbTransport>(h, true, live_);
}

void LibusbBus::sleep(double seconds)
{
    std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
}

} // namespace niusb
