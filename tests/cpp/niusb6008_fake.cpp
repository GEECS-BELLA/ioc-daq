/* The USB-6008 driver against its fake (decision 13): open, the loopbacks,
 * the refusals, an unplug and replug. Prints one "check <name> <ok|FAIL>
 * <detail>" line per check. */
#include <chrono>
#include <cmath>
#include <iostream>

#include "NiUsb6008.h"
#include "NiUsb6008Fake.h"

using namespace niusb;
using namespace niusb::n6008;

static int failures = 0;

static void check(const std::string &name, bool ok, const std::string &detail = "")
{
    std::cout << "check " << name << " " << (ok ? "ok" : "FAIL") << " " << detail << "\n";
    failures += !ok;
}

/* Refused by the driver: an Error that is not a TransportError (a USB
 * failure would pass for the wrong reason) and, if given, says what. */
template <class F> static void refused(const std::string &name, F f, const std::string &says = "")
{
    try {
        f();
        check(name, false, "not refused");
    } catch (const TransportError &e) {
        check(name, false, std::string("a transport error: ") + e.what());
    } catch (const Error &e) {
        check(name, std::string(e.what()).find(says) != std::string::npos, e.what());
    }
}

static bool outputsTouched(const Fake6008Unit &u)
{
    for (const Bytes &m : u.received)
        if (m.size() >= 10 && (m[9] == SUB_AO || (m[9] == SUB_DIO && m[7] != 0x0E)))
            return true;
    return false;
}

int main()
{
    Fake6008Bus bus;
    auto a = bus.add("FAKEA");
    auto b = bus.add("FAKEB");
    a->ao[0] = 1234;  // the state an earlier IOC left
    a->p1Latch = 0x5;
    a->p1Dir = 0xF;

    Device6008 d(bus, "FAKEA");
    d.open();
    check("open-by-serial", d.serial() == "FAKEA" && a->received.size() == 16 && b->received.empty());
    check("calibration-read", d.calMemory() == fake6008Memory() &&
                                  std::fabs(d.calibration().rse[0].second - 10.6265306) < 1e-6);
    check("open-leaves-outputs", !outputsTouched(*a) && a->ao[0] == 1234 && a->p1Latch == 0x5);
    check("p1-state-read", d.diRead(1) == 0x5 && d.diRead(0) == 0xF5);

    for (double v : {1.0, 2.5, 4.5, 0.0}) {
        d.aoWrite(0, v);
        double got = d.aiRead(0, 10.0, Term::RSE, 3);
        check("ao0-ai0 " + std::to_string(v), std::fabs(got - v) < 6e-3, std::to_string(got));
        d.aoWrite(1, v);
        got = d.aiRead(1, 10.0, Term::RSE, 1);
        check("ao1-ai1 " + std::to_string(v), std::fabs(got - v) < 6e-3, std::to_string(got));
    }
    check("ao-clamped-to-5V", d.aoCode(0, 7.0) == d.aoCode(0, 5.0) && d.aoCode(1, -2.0) == d.aoCode(1, 0.0));
    check("ao1-0V-below-code-0", d.aoCode(1, 0.0) == -5, std::to_string(d.aoCode(1, 0.0)));
    check("ai5-0.5V", std::fabs(d.aiRead(5, 10.0, Term::RSE) - 0.5) < 6e-3);
    d.aoWrite(0, 2.5);
    double diff = d.aiRead(0, 10.0, Term::DIFF, 2);   // AI0 - AI4 = 2.5 - 0.4
    check("diff-ai0-ai4", std::fabs(diff - 2.1) < 3e-3, std::to_string(diff));
    d.aoWrite(1, 0.5);
    diff = d.aiRead(1, 1.0, Term::DIFF);               // AI1 - AI5 = 0.5 - 0.5
    check("diff-1V-range", std::fabs(diff) < 1e-3, std::to_string(diff));

    // DO on P1, looped back to P0.0-3
    size_t before = a->received.size();
    d.doWrite(0x0);
    check("first-do-latches-then-directs", a->received.size() - before == 4 && a->received[before + 1][7] == 0x0F &&
                                               a->received[before + 1][17] == 0x5 && a->received[before + 2][7] == 0x12);
    before = a->received.size();
    for (int v = 0; v < 16; v++) {
        d.doWrite(uint8_t(v));
        if (d.diRead(0) != (0xF0 | v) || d.diRead(1) != v)
            check("do-p0-loopback " + std::to_string(v), false);
    }
    check("do-p0-loopback-all", a->received.size() - before == 16 * 4);
    d.doWrite(0x0);
    check("do-masked", d.doWrite(0x1, 0x1) == 0x1 && d.doWrite(0x8, 0x8) == 0x9 && d.doWrite(0x0, 0x1) == 0x8);

    std::vector<uint16_t> w = d.aiTimedWords({0, 1}, 5000, 500, 10.0, Term::RSE);
    double s0 = 0, s1 = 0;
    for (size_t i = 0; i < w.size(); i += 2) {
        s0 += d.scale(w[i], 10.0, Term::RSE, 0);
        s1 += d.scale(w[i + 1], 10.0, Term::RSE, 1);
    }
    check("timed-count", w.size() == 1000, std::to_string(w.size()));
    check("timed-loopbacks", std::fabs(s0 / 500 - 2.5) < 6e-3 && std::fabs(s1 / 500 - 0.5) < 6e-3,
          std::to_string(s0 / 500) + " " + std::to_string(s1 / 500));
    check("timed-divisor", Device6008::timedDivisor(1, 1000) == 24000 && Device6008::timedDivisor(2, 5000) == 4800);

    refused("rse-only-10V", [&] { d.aiRead(0, 5.0, Term::RSE); });
    refused("diff-channel-4", [&] { d.aiRead(4, 10.0, Term::DIFF); });
    refused("diff-bad-range", [&] { d.aiRead(0, 0.2, Term::DIFF); });
    refused("rse-channel-8", [&] { d.aiRead(8, 10.0, Term::RSE); });
    refused("aggregate-rate", [&] { d.aiTimedWords({0, 1}, 6000, 10, 10.0, Term::RSE); });
    refused("divisor-beyond-16-bits", [&] { d.aiTimedWords({0}, 100, 10, 10.0, Term::RSE); });
    refused("no-ao2", [&] { d.aoWrite(2, 1.0); });
    refused("do-5-lines", [&] { d.doWrite(0x10); });
    refused("missing-serial", [&] {
        Device6008 none(bus, "NOSUCH");
        none.open();
    }, "no NI USB-6008 with serial NOSUCH found");

    // A timed scan whose samples never come: refused at its deadline after the
    // teardown, the unit stays open, and the next on-demand read is prompt.
    a->stallTimed = true;
    size_t sentBefore = a->received.size();
    auto t0 = std::chrono::steady_clock::now();
    refused("stalled-scan", [&] { d.aiTimedWords({0}, 1000, 10, 10.0, Term::RSE); }, "by the deadline");
    a->stallTimed = false;
    bool torn = false;
    for (size_t i = sentBefore; i < a->received.size(); i++)
        torn = torn || (a->received[i][7] == 0x0D && a->received[i][9] == SUB_AI_TIMING);
    check("stalled-scan-torn-down", torn && d.isOpen());
    auto t1 = std::chrono::steady_clock::now();
    double v = d.aiRead(0, 10.0, Term::RSE);
    double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t1).count();
    check("read-after-stalled-scan", std::fabs(v - 2.5) < 6e-3 && dt < 1.0,
          std::to_string(v) + " V in " + std::to_string(dt) + " s, scan " +
              std::to_string(std::chrono::duration<double>(t1 - t0).count()) + " s");

    // Unplugged: a transport error; the unit cannot be found until it is back.
    a->present = false;
    try {
        d.aiRead(0, 10.0, Term::RSE);
        check("unplug-is-transport-error", false);
    } catch (const TransportError &e) {
        check("unplug-is-transport-error", true, e.what());
    }
    d.close();
    refused("reopen-while-unplugged", [&] { d.open(); }, "no NI USB-6008 with serial FAKEA found");
    check("failed-open-leaves-nothing-open", !d.isOpen());
    a->present = true;
    a->replies.push_back(fromHex("0001000c0008010000000002"));  // a stale reply from before the pull
    a->data = {0x12, 0x34};
    size_t sent = a->received.size();
    int16_t aoBefore = a->ao[0];
    d.open();
    bool touched = false;
    for (size_t i = sent; i < a->received.size(); i++)
        touched = touched || a->received[i][9] == SUB_AO || a->received[i][9] == SUB_DIO;
    check("reopen-reads-only", d.isOpen() && !touched && a->ao[0] == aoBefore);
    check("stale-reply-drained", std::fabs(d.aiRead(0, 10.0, Term::RSE) - 2.5) < 6e-3);
    size_t n = a->received.size();
    d.doWrite(0x2, 0x2);
    check("reopen-redoes-direction", a->received.size() - n == 4);

    Device6008 first(bus, "");
    first.open();
    check("first-unit", first.serial() == "FAKEA");

    std::cout << (failures ? "FAILED" : "PASSED") << "\n";
    return failures ? 1 : 0;
}
