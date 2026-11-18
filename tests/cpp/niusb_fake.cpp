/* The driver against the fake (decision 13): two units on one bus, one
 * blank (loaded with filler firmware) and one already loaded; then the
 * loopbacks. Prints one "check <name> <ok|FAIL> <detail>" line per check. */
#include <cmath>
#include <iostream>

#include "NiUsb621x.h"
#include "NiUsbFake.h"

using namespace niusb;

static int failures = 0;

static void check(const std::string &name, bool ok, const std::string &detail = "")
{
    std::cout << "check " << name << " " << (ok ? "ok" : "FAIL") << " " << detail << "\n";
    failures += !ok;
}

int main()
{
    FakeBus bus;
    auto blank = bus.add(0x7270, "FAKE6211", false);
    auto ready = bus.add(0x726F, "FAKE6210", true);
    Firmware fw;
    fw.loader.assign(1284, 1);
    fw.main16k.assign(16384, 2);
    fw.bank0.assign(10438, 3);
    fw.bank1.assign(3601, 4);
    fw.fpga0.assign(78422, 5);
    fw.fpga1.assign(283776, 6);

    Device d6211(bus, "FAKE6211", &fw);
    d6211.open();
    check("blank-loaded", blank->loaded && blank->renumerations == 1 && blank->fpga0 && blank->fpga1);
    check("model", std::string(d6211.model().name) == "USB-6211", d6211.model().name);

    Device d6210(bus, "FAKE6210", nullptr);
    d6210.open();
    check("loaded-unit-no-firmware-needed", std::string(d6210.model().name) == "USB-6210");
    check("6210-ai3", std::fabs(d6210.aiRead(3, 10.0, Mode::RSE, 4) - 0.3) < 1e-3);

    Device none(bus, "NOSUCH", nullptr);
    try {
        none.open();
        check("missing-serial-refused", false);
    } catch (const Error &e) {
        check("missing-serial-refused", true, e.what());
    }

    for (double v : {2.5, -7.5, 0.0}) {
        d6211.aoWrite(1, v);
        double got = d6211.aiRead(2, 10.0, Mode::RSE, 3);
        check("ao1-ai2 " + std::to_string(v), std::fabs(got - v) < 2e-3, std::to_string(got));
    }
    d6211.aoWrite(0, 1.0);
    check("ao0-ai1-1V-range", std::fabs(d6211.aiRead(1, 1.0, Mode::RSE, 2) - 1.0) < 1e-3);
    for (int v = 0; v < 16; v++) {
        d6211.doWrite(uint8_t(v));
        uint32_t w = d6211.pfiRead();
        if (((w >> 4) & 0xF) != uint32_t(v) || (w & 1) != uint32_t(v & 1)) {
            check("do-readback " + std::to_string(v), false, std::to_string(w));
        }
    }
    check("do-readback-all", true);
    d6211.aoWrite(1, 3.0);
    std::vector<int16_t> c = d6211.aiTimedCodes({0, 2}, 10000, 500, 10.0, Mode::RSE);
    double a0 = 0, a2 = 0;
    for (size_t i = 0; i < c.size(); i += 2) {
        a0 += d6211.scale(c[i], 10.0);
        a2 += d6211.scale(c[i + 1], 10.0);
    }
    check("timed-count", c.size() == 1000, std::to_string(c.size()));
    check("timed-ai2", std::fabs(a2 / 500 - 3.0) < 2e-3, std::to_string(a2 / 500));
    check("timed-ai0", std::fabs(a0 / 500) < 2e-3, std::to_string(a0 / 500));
    try {
        d6210.aoWrite(0, 1.0);
        check("6210-no-ao", false);
    } catch (const Error &) {
        check("6210-no-ao", true);
    }
    try {
        d6211.aiRead(0, 2.0, Mode::RSE);
        check("bad-range-refused", false);
    } catch (const Error &) {
        check("bad-range-refused", true);
    }
    try {
        d6211.aiRead(9, 10.0, Mode::DIFF);
        check("diff-channel-refused", false);
    } catch (const Error &) {
        check("diff-channel-refused", true);
    }
    check("fresh-after-blank-load", d6211.freshlyLoaded());
    d6211.open();
    check("not-fresh-on-reopen", !d6211.freshlyLoaded());
    try {
        d6211.aiTimedCodes({0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}, 100000, 10, 10.0, Mode::RSE);
        check("aggregate-rate-refused", false);
    } catch (const Error &e) {
        check("aggregate-rate-refused", true, e.what());
    }

    // A unit with its firmware loaded but not its FPGAs (a load stopped half
    // way): without NI's firmware the open fails and nothing stays open;
    // with it, the open brings the FPGAs up.
    FakeBus bus2;
    auto half = bus2.add(0x7270, "HALF", true);
    half->fpga0 = half->fpga1 = false;
    Device noFw(bus2, "HALF", nullptr);
    try {
        noFw.open();
        check("half-loaded-no-firmware-refused", false);
    } catch (const Error &e) {
        check("half-loaded-no-firmware-refused", true, e.what());
    }
    check("failed-open-leaves-nothing-open", !noFw.isOpen());
    try {
        noFw.aiRead(3, 10.0, Mode::RSE);
        check("no-reading-after-failed-open", false);
    } catch (const Error &) {
        check("no-reading-after-failed-open", true);
    }
    Device withFw(bus2, "HALF", &fw);
    withFw.loadLock = "/dev/null";
    withFw.open();
    check("half-loaded-brought-up", half->fpga0 && half->fpga1 &&
                                        std::fabs(withFw.aiRead(3, 10.0, Mode::RSE) - 0.3) < 1e-3);

    // A blank unit loaded under the lock.
    FakeBus bus3;
    auto b3 = bus3.add(0x726F, "LOCKED", false);
    Device locked(bus3, "LOCKED", &fw);
    locked.loadLock = "/dev/null";
    locked.open();
    check("blank-load-under-lock", b3->loaded && locked.freshlyLoaded());

    std::cout << (failures ? "FAILED" : "PASSED") << "\n";
    return failures ? 1 : 0;
}
