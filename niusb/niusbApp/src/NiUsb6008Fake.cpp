#include "NiUsb6008Fake.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

namespace niusb {

using namespace n6008;

const Bytes &fake6008Memory()
{
    // tests/fixtures/daqmx-6008.json, section "cal": the unit's memory as
    // NI-DAQmx read it at plug-in (its data, not NI software)
    static const Bytes m = fromHex(
        "100120141016000040200dbd40a206ccc1a6bb1a41a72f32c1a6d37141a72784"
        "c1a6db8441a72d2ac1a6cb4541a72c3fc126aa3241273e46c126d27541272375"
        "c126dcc841272887c126c93a412730cbc0a698c140a75bebc0a6dbfe40a72ea1"
        "c0a6e53d40a73013c0a6c31f40a748dac08531aa4085e2aec08570d34085b0fc"
        "c0857aa14085b22bc08554de4085ce9ac02659174027938cc026d97840272761"
        "c026e56e4027245dc026a17840275608c00500de40062264c0057b794005b924"
        "c00586984005b309c00546e64005e75cbfa5f37c3fa80434bfa6e6173fa72af3"
        "bfa6f8d53fa71cc7bfa67fcd3fa78f0abf84aa723f86a9b0bf8596943f85cb8b"
        "bf85ad033f85c0a5bf8531133f862f10c123c281412a0645c1241127412a1f1f"
        "c1240454412a08efc1240b85412a1726c1240f4b412a15dec1240967412a1a4f"
        "c123f54f412a0ebcc1240bc7412a13e3ba950df440a1c58c3bbbf85040a14ad2"
        "ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff"
        "ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff"
        "ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff"
        "ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff"
        "ffffffffffffffffffffffffffffffff717a717a000100013923717a019cdc74");
    return m;
}

Fake6008Unit::Fake6008Unit(std::string s) : serial(std::move(s)), mem(fake6008Memory()), cal(Calibration::parse(mem)) {}

double Fake6008Unit::input(int chan) const
{
    if (chan == 0 || chan == 1) {  // AO0 -> AI0, AO1 -> AI1
        auto c = cal.ao[chan];
        return c.first + ao[chan] * (c.second - c.first) / 4095;
    }
    return 0.1 * chan;
}

uint16_t Fake6008Unit::word(int chan) const
{
    // the inverse of the driver's scaling, to the unit's 12 bits
    auto q = [](double x) { return 16 * std::floor(x / 16 + 0.5); };
    bool diff = std::find(aiModes.begin(), aiModes.end(), 0x00) != aiModes.end();
    if (diff) {
        int r = 0;
        const std::vector<double> &rs = diffRanges();
        for (size_t i = 0; i < rs.size(); i++)
            if (std::lround(rs[i] * 1000) == rangeMv)
                r = int(i);
        auto c = cal.diff[r][chan & 3];
        double v = input(chan & 3) - input((chan & 3) + 4);
        double w = q((v - c.first) * 65536 / (c.second - c.first) - 0.5);
        return uint16_t(std::max(0.0, std::min(65520.0, w)));
    }
    auto c = cal.rse[chan & 7];
    double w = q((input(chan & 7) - c.second) * 32768 / (c.second - c.first) - 0.5);
    return uint16_t(int16_t(std::max(-32768.0, std::min(0.0, w))));
}

static uint16_t be16(const Bytes &b, size_t at)
{
    return uint16_t(b[at] << 8 | b[at + 1]);
}

static Bytes reply(const Bytes &m, const Bytes &data = {})
{
    Bytes r = {m[0], m[1], 0, 0, 0, 0, 0x01, 0, 0, 0, 0, 0x02};
    r.insert(r.end(), data.begin(), data.end());
    uint16_t n = uint16_t(r.size());
    r[2] = uint8_t(n >> 8);
    r[3] = uint8_t(n);
    r[4] = uint8_t((n - 4) >> 8);
    r[5] = uint8_t(n - 4);
    return r;
}

void Fake6008Unit::command(const Bytes &m)
{
    received.push_back(m);
    if (m.size() < 12 || m[8] != 0x02)
        return;
    uint8_t kind = m[6], verb = m[7], sub = m[9];
    uint16_t idx = be16(m, 10);
    Bytes p(m.begin() + 12, m.end());
    auto push = [&](uint16_t w) {
        data.push_back(uint8_t(w >> 8));
        data.push_back(uint8_t(w));
    };
    if (kind == 0) {
        if (sub == SUB_AI_DATA && verb == 0x14 && p.size() >= 4 && !aiChans.empty())
            for (uint32_t i = 0; i < be32(&p[0]); i++)
                push(word(aiChans[0]));
        return;  // a timed read's announce (12 02 03): the samples come at the start
    }
    auto ok = [&](const Bytes &d = {}) { replies.push_back(reply(m, d)); };
    auto p1 = [&] { return uint8_t((p1Latch & p1Dir) | (0x0F & ~p1Dir)); };
    if (sub == SUB_CAL && verb == 0x14 && p.size() >= 6) {
        uint32_t a = be32(&p[0]);
        uint16_t n = be16(p, 4);
        Bytes d = {uint8_t((n + 2) >> 8), uint8_t(n + 2)};
        for (uint32_t i = a; i < a + n || (12 + d.size()) % 4; i++)
            d.push_back(i < mem.size() ? mem[i] : 0);
        ok(d);
    } else if (sub == SUB_AO && verb == 0x0E && idx < 2 && p.size() >= 2) {
        ao[idx] = int16_t(be16(p, 0));
        aoWritten[idx] = true;
        ok();
    } else if (sub == SUB_DIO && verb == 0x0E && p.size() >= 3) {
        // P0: P0.0-3 wired to P1.0-3, P0.4-7 pulled up; P1: f0 | its lines
        ok({0, 3, uint8_t(0xF0 | p1()), 0});
    } else if (sub == SUB_DIO && verb == 0x0F && p.size() >= 6) {
        if (p[2] == 1)
            p1Latch = p[5] & 0x0F;
        ok();
    } else if (sub == SUB_DIO && verb == 0x12 && p.size() >= 4) {
        p0Dir = p[2];
        p1Dir = p[3] & 0x0F;
        ok();
    } else if (sub == SUB_AI_CHAN && verb == 0x0F && p.size() >= 2) {
        aiChans.clear();
        for (size_t i = 2; i + 1 < be16(p, 0) && i + 1 < p.size(); i += 2)
            aiChans.push_back(be16(p, i));
        ok();
    } else if (sub == SUB_AI_CHAN && verb == 0x0E && p.size() >= 2) {
        aiModes.assign(p.begin() + 2, p.begin() + std::min<size_t>(p.size(), be16(p, 0)));
        ok();
    } else if (sub == SUB_AI_CHAN && verb == 0x10 && p.size() >= 4) {
        rangeMv = int32_t(be32(&p[0]));
        ok();
    } else if (sub == SUB_AI_TIMING && verb == 0x0E && p.size() >= 4) {
        timed = be32(&p[0]) == 0x01000000;
        ok();
    } else if (sub == SUB_AI_TIMING && verb == 0x0F && p.size() >= 4) {
        count = be32(&p[0]);
        ok();
    } else if (sub == SUB_AI_TIMING && verb == 0x0A) {
        if (timed && !stallTimed)
            for (uint32_t i = 0; i < count; i++)
                for (int c : aiChans)
                    push(word(c));
        ok();
    } else if (sub == SUB_AI_DATA && verb == 0x13) {
        ok({0, 0, 0, 0});
    } else {
        ok();
    }
}

namespace {

class Fake6008Transport : public Transport {
public:
    Fake6008Transport(Fake6008Bus &bus, std::shared_ptr<Fake6008Unit> u) : bus_(bus), u_(std::move(u)) {}

    void ctrlOut(uint8_t, uint16_t, uint16_t, const Bytes &, int) override
    {
        throw TransportError("fake 6008: no vendor control requests");
    }
    Bytes ctrlIn(uint8_t, uint16_t, uint16_t, uint16_t, int) override
    {
        throw TransportError("fake 6008: no vendor control requests");
    }

    void write(uint8_t ep, const Bytes &data, int) override
    {
        std::lock_guard<std::mutex> g(bus_.lock);
        gone();
        if (ep != EP_CMD)
            throw TransportError("fake 6008: write to ep " + std::to_string(ep));
        u_->command(data);
    }

    Bytes read(uint8_t ep, size_t maxLen, int timeoutMs) override
    {
        std::unique_lock<std::mutex> g(bus_.lock);
        gone();
        if (ep == EP_REPLY) {
            if (u_->replies.empty())
                throw Timeout("fake 6008: no reply pending");
            Bytes r = u_->replies.front();
            u_->replies.pop_front();
            return r;
        }
        if (ep == EP_SAMPLES) {
            if (u_->data.empty()) {
                g.unlock();  // a read with nothing to come waits its timeout, as on the unit
                std::this_thread::sleep_for(std::chrono::milliseconds(std::min(timeoutMs, 100)));
                throw Timeout("fake 6008: no samples pending");
            }
            size_t n = std::min(maxLen, u_->data.size());
            Bytes r(u_->data.begin(), u_->data.begin() + n);
            u_->data.erase(u_->data.begin(), u_->data.begin() + n);
            return r;
        }
        throw TransportError("fake 6008: read from ep " + std::to_string(ep));
    }

private:
    void gone() const
    {
        if (!u_->present)
            throw TransportError("fake 6008: LIBUSB_ERROR_NO_DEVICE (unplugged)");
    }
    Fake6008Bus &bus_;
    std::shared_ptr<Fake6008Unit> u_;
};

} // namespace

std::shared_ptr<Fake6008Unit> Fake6008Bus::add(const std::string &serial)
{
    std::lock_guard<std::mutex> g(lock);
    units.push_back(std::make_shared<Fake6008Unit>(serial));
    return units.back();
}

std::vector<UnitInfo> Fake6008Bus::list()
{
    std::lock_guard<std::mutex> g(lock);
    std::vector<UnitInfo> out;
    for (size_t i = 0; i < units.size(); i++) {
        if (!units[i]->present)
            continue;
        UnitInfo u;
        u.bus = 1;
        u.ports = {int(i) + 1};
        u.pid = PID;
        u.serial = units[i]->serial;
        out.push_back(u);
    }
    return out;
}

std::unique_ptr<Transport> Fake6008Bus::open(const UnitInfo &unit)
{
    std::lock_guard<std::mutex> g(lock);
    if (unit.ports.empty() || unit.ports[0] < 1 || size_t(unit.ports[0]) > units.size() ||
        !units[unit.ports[0] - 1]->present)
        throw TransportError("fake 6008: no such unit");
    return std::make_unique<Fake6008Transport>(*this, units[unit.ports[0] - 1]);
}

} // namespace niusb
