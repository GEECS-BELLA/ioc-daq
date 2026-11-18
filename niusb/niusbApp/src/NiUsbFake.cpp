#include "NiUsbFake.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace niusb {

static void putFloat(Bytes &e, unsigned at, float f)
{
    uint32_t u;
    std::memcpy(&u, &f, 4);
    for (int i = 0; i < 4; i++)
        e[at + i] = uint8_t(u >> (24 - 8 * i));
}

static const double GAIN[7] = {3.29e-4, 1.645e-4, 6.44e-5, 3.3e-5, 1.61e-5, 6.59e-6, 3.22e-6};
static const double AO_GAIN = 3214.0, AO_OFFSET = 1.0;

Bytes fakeEeprom(uint16_t pid)
{
    Bytes e(EEPROM_SIZE, 0);
    e[0] = uint8_t(pid >> 8);
    e[1] = uint8_t(pid);
    for (int i = 0; i < 7; i++)
        putFloat(e, SELF_CAL_TABLE + 8 * i, float(GAIN[i]));
    if (const Model *m = modelByPid(pid))
        for (unsigned a : m->aoCal) {
            putFloat(e, a, float(AO_GAIN));
            putFloat(e, a + 4, float(AO_OFFSET));
        }
    return e;
}

FakeUnit::FakeUnit(uint16_t p, std::string s, bool l)
    : pid(p), serial(std::move(s)), loaded(l), fpga0(l), fpga1(l), eeprom(fakeEeprom(p))
{
}

double FakeUnit::input(int chan) const
{
    if (chan == 1 || chan == 2) {   // AO0 -> AI1, AO1 -> AI2
        if (!modelByPid(pid) || modelByPid(pid)->aoCal.empty())
            return 0.0;
        return (ao[chan - 1] - AO_OFFSET) / AO_GAIN;
    }
    return 0.1 * chan;
}

static uint16_t be16(const Bytes &b, size_t at)
{
    return uint16_t(b[at] << 8 | b[at + 1]);
}

static Bytes reply(const Bytes &m, uint8_t status, const Bytes &data = {})
{
    Bytes r = {m[0], m[1], 0, 0, 0, 0, 0x01, 0, 0, 0, 0, status};
    r.insert(r.end(), data.begin(), data.end());
    uint16_t n = uint16_t(r.size());
    r[2] = uint8_t(n >> 8);
    r[3] = uint8_t(n);
    r[4] = uint8_t((n - 4) >> 8);
    r[5] = uint8_t(n - 4);
    return r;
}

static Bytes word(uint32_t v)
{
    return {uint8_t(v >> 24), uint8_t(v >> 16), uint8_t(v >> 8), uint8_t(v)};
}

void FakeUnit::command(const Bytes &m)
{
    received.push_back(m);
    if (m.size() < 12)
        return;
    uint8_t kind = m[6], verb = m[7], sub = m[9];
    uint16_t idx = be16(m, 10);
    Bytes p(m.begin() + 12, m.end());
    if (kind == 0)
        return;  // an announce: the following transfer's length; nothing to answer
    auto ok = [&](const Bytes &d = {}) { replies.push_back(reply(m, 2, d)); };
    if (sub == 0xFE || sub == 0xFF) {
        if (verb == 0x08) {
            fpgaArmed = sub;
            fpgaBytes = 0;
            ok(word(0));
        } else if (verb == 0x09) {
            bool done = sub == 0xFE ? fpga0 : fpga1;
            if (sub == 0xFE)
                ok(word(done ? 0x01000000 : 0));
            else if (done)
                ok();
            else
                ok(word(0));
        } else {
            ok();
        }
        return;
    }
    if (sub == 0x05 && verb == 0x08) {
        if (!(fpga0 && fpga1)) {
            replies.push_back(reply(m, 6, word(0xFFFF3BB0)));
            return;
        }
        uint32_t off = be32(&p[0]), n = be32(&p[4]);
        Bytes d = {uint8_t((n + 2) >> 8), uint8_t(n + 2)};
        d.insert(d.end(), eeprom.begin() + off, eeprom.begin() + std::min<size_t>(off + n, eeprom.size()));
        while ((12 + d.size()) % 4)
            d.push_back(0);
        ok(d);
        return;
    }
    if (sub == 0x20 && verb == 0x0A && p.size() >= 4 && idx < 2) {
        ao[idx] = int16_t(be16(p, 2));
        ok();
        return;
    }
    if (sub == 0x12 && verb == 0x0A && p.size() >= 4) {
        uint16_t count = be16(p, 0);
        aiChans.clear();
        for (size_t i = 2; i + 1 < count; i += 2) {
            uint16_t w = be16(p, i);
            aiChans.push_back(w & 0xF);
            aiGain = (w >> 9) & 7;
        }
        ok();
        return;
    }
    if (sub == 0x13 && verb == 0x10 && p.size() >= 24) {
        timedMode = be32(&p[8]);
        timedN = be32(&p[16]) + 1;
        ok();
        return;
    }
    if (sub == 0x13 && verb == 0x0C) {
        static const int slot[8] = {0, 1, 0, 0, 3, 5, 0, 0};
        double g = GAIN[slot[aiGain]];
        auto code = [&](int chan) {
            double c = std::round(input(chan) / g);
            int16_t v = int16_t(std::max(-32768.0, std::min(32767.0, c)));
            data.push_back(uint8_t(v));
            data.push_back(uint8_t(uint16_t(v) >> 8));
        };
        if (timedMode == 2) {
            for (uint32_t i = 0; i < timedN; i++)
                for (int c : aiChans)
                    code(c);
        } else if (!aiChans.empty()) {
            code(aiChans[0]);
        }
        ok();
        return;
    }
    if (sub == 0x28 && verb == 0x08 && idx == 2) {
        ok(word(uint32_t(doValue) << DO_SHIFT | (doValue & 1)));
        return;
    }
    if (sub == 0x28 && verb == 0x09 && idx == 3 && p.size() >= 8) {
        doValue = uint8_t((be32(&p[4]) >> DO_SHIFT) & 0xF);
        ok();
        return;
    }
    ok();
}

namespace {

class FakeTransport : public Transport {
public:
    FakeTransport(FakeBus &bus, std::shared_ptr<FakeUnit> u) : bus_(bus), u_(std::move(u)) {}

    void ctrlOut(uint8_t req, uint16_t value, uint16_t, const Bytes &data, int) override
    {
        std::lock_guard<std::mutex> g(bus_.lock);
        if (u_->loaded)
            throw TransportError("fake: vendor request to a loaded unit");
        if (req == 0xA0 && value == 0xE600) {
            u_->cpucs = data.empty() ? 0 : data[0];
            if (u_->cpucs == 0 && u_->mainWritten) {   // the main image runs: renumerate
                u_->loaded = true;
                u_->renumerations++;
            }
            return;
        }
        u_->fx2Writes.push_back({uint32_t(req) << 16 | value, data});
        if (req == 0xA4 || req == 0xA5)
            u_->banksWritten = true;
        if (req == 0xA0 && u_->banksWritten && u_->cpucs == 1)
            u_->mainWritten = true;
    }

    Bytes ctrlIn(uint8_t req, uint16_t value, uint16_t, uint16_t length, int) override
    {
        std::lock_guard<std::mutex> g(bus_.lock);
        if (req == 0xB0)
            return {0x70, 0x72};
        for (auto it = u_->fx2Writes.rbegin(); it != u_->fx2Writes.rend(); ++it)
            if (it->first == (uint32_t(req) << 16 | value))
                return Bytes(it->second.begin(), it->second.begin() + std::min<size_t>(length, it->second.size()));
        return Bytes(length, 0);
    }

    void write(uint8_t ep, const Bytes &data, int) override
    {
        std::lock_guard<std::mutex> g(bus_.lock);
        if (!u_->loaded)
            throw TransportError("fake: bulk write to a blank unit");
        if (ep == EP_CMD) {
            u_->command(data);
        } else if (ep == EP_FPGA) {
            u_->fpgaBytes += data.size();
            if (u_->fpgaArmed == 0xFE)
                u_->fpga0 = true;
            if (u_->fpgaArmed == 0xFF)
                u_->fpga1 = true;
        } else {
            throw TransportError("fake: write to ep " + std::to_string(ep));
        }
    }

    Bytes read(uint8_t ep, size_t maxLen, int) override
    {
        std::lock_guard<std::mutex> g(bus_.lock);
        if (ep == EP_REPLY) {
            if (u_->replies.empty())
                throw Timeout("fake: no reply pending");
            Bytes r = u_->replies.front();
            u_->replies.pop_front();
            return r;
        }
        if (ep == EP_DATA) {
            if (u_->data.empty())
                throw Timeout("fake: no AI data pending");
            size_t n = std::min(maxLen, u_->data.size());
            Bytes r(u_->data.begin(), u_->data.begin() + n);
            u_->data.erase(u_->data.begin(), u_->data.begin() + n);
            return r;
        }
        throw TransportError("fake: read from ep " + std::to_string(ep));
    }

private:
    FakeBus &bus_;
    std::shared_ptr<FakeUnit> u_;
};

} // namespace

std::shared_ptr<FakeUnit> FakeBus::add(uint16_t pid, const std::string &serial, bool loaded)
{
    std::lock_guard<std::mutex> g(lock);
    units.push_back(std::make_shared<FakeUnit>(pid, serial, loaded));
    return units.back();
}

std::vector<UnitInfo> FakeBus::list()
{
    std::lock_guard<std::mutex> g(lock);
    std::vector<UnitInfo> out;
    for (size_t i = 0; i < units.size(); i++) {
        UnitInfo u;
        u.bus = 1;
        u.ports = {int(i) + 1};
        u.pid = units[i]->loaded ? units[i]->pid : PID_BLANK;
        u.serial = units[i]->loaded ? units[i]->serial : "";
        out.push_back(u);
    }
    return out;
}

std::unique_ptr<Transport> FakeBus::open(const UnitInfo &unit)
{
    std::lock_guard<std::mutex> g(lock);
    if (unit.ports.empty() || unit.ports[0] < 1 || size_t(unit.ports[0]) > units.size())
        throw TransportError("fake: no such unit");
    return std::make_unique<FakeTransport>(*this, units[unit.ports[0] - 1]);
}

} // namespace niusb
