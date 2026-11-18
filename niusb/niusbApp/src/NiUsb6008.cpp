#include "NiUsb6008.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace niusb {
namespace n6008 {

const std::vector<double> &diffRanges()
{
    static const std::vector<double> r = {20.0, 10.0, 5.0, 4.0, 2.5, 2.0, 1.25, 1.0};
    return r;
}

int diffRangeIndex(double range)
{
    const std::vector<double> &r = diffRanges();
    for (size_t i = 0; i < r.size(); i++)
        if (std::fabs(r[i] - range) < 1e-9)
            return int(i);
    throw Error("the 6008's DIFF ranges are +-20, 10, 5, 4, 2.5, 2, 1.25 and 1 V, not " + std::to_string(range));
}

static void put16(Bytes &b, uint16_t v)
{
    b.push_back(uint8_t(v >> 8));
    b.push_back(uint8_t(v));
}

static void put32(Bytes &b, uint32_t v)
{
    put16(b, uint16_t(v >> 16));
    put16(b, uint16_t(v));
}

static Bytes u32(uint32_t v)
{
    Bytes b;
    put32(b, v);
    return b;
}

static void pad4(Bytes &b)
{
    while (b.size() % 4)
        b.push_back(0);
}

Bytes msg6008(uint8_t verb, uint8_t subsys, const Bytes &payload, uint16_t index, uint8_t kind, uint16_t ch)
{
    Bytes out;
    uint16_t n = uint16_t(12 + payload.size());
    put16(out, ch);
    put16(out, n);
    put16(out, uint16_t(n - 4));
    out.insert(out.end(), {kind, verb, 0x02, subsys});
    put16(out, index);
    out.insert(out.end(), payload.begin(), payload.end());
    return out;
}

Bytes calRead(uint32_t addr, uint16_t n)
{
    Bytes p;
    put32(p, addr);
    put16(p, n);
    put16(p, 0);
    return msg6008(0x14, SUB_CAL, p);
}

/* u16 byte count (2 + 2 per entry), the entries as u16, unpadded. */
static Bytes list(const std::vector<int> &v)
{
    Bytes b;
    put16(b, uint16_t(2 + 2 * v.size()));
    for (int c : v)
        put16(b, uint16_t(c));
    return b;
}

std::vector<Bytes> aiConfig(const std::vector<int> &chans, double range, Term mode)
{
    if (chans.empty())
        throw Error("an AI channel list needs a channel");
    Bytes cl = list(chans);
    pad4(cl);
    Bytes modes;
    put16(modes, uint16_t(2 + chans.size()));
    for (size_t i = 0; i < chans.size(); i++)
        modes.push_back(mode == Term::RSE ? 0x02 : 0x00);
    pad4(modes);
    // The range applies to list entries 0..n-1, named by position, not by channel.
    int32_t mv = int32_t(std::lround(range * 1000));
    Bytes rg;
    put32(rg, uint32_t(mv));
    put32(rg, uint32_t(-mv));
    rg.push_back(0xFD);
    rg.push_back(0xFD);
    std::vector<int> pos(chans.size());
    for (size_t i = 0; i < pos.size(); i++)
        pos[i] = int(i);
    Bytes pl = list(pos);
    rg.insert(rg.end(), pl.begin(), pl.end());
    pad4(rg);
    return {msg6008(0x0F, SUB_AI_CHAN, cl), msg6008(0x0E, SUB_AI_CHAN, modes), msg6008(0x10, SUB_AI_CHAN, rg)};
}

std::vector<Bytes> aiOnDemand(int chan, double range, Term mode, uint32_t n)
{
    std::vector<Bytes> m = aiConfig({chan}, range, mode);
    m.insert(m.end(), {msg6008(0x0E, SUB_AI_TIMING, u32(0)), msg6008(0x0F, SUB_AI_TIMING, u32(1)),
                       msg6008(0x13, SUB_AI_TIMING), msg6008(0x15, SUB_AI_TIMING), msg6008(0x18, SUB_AI_TIMING),
                       msg6008(0x0F, SUB_AI_DATA), msg6008(0x09, SUB_AI_DATA), msg6008(0x09, SUB_AI_TIMING),
                       msg6008(0x14, SUB_AI_DATA, u32(n), 0, 0)});
    return m;
}

std::vector<Bytes> aiTimed(const std::vector<int> &chans, double range, Term mode, uint32_t divisor, uint32_t n)
{
    std::vector<Bytes> m = aiConfig(chans, range, mode);
    m.insert(m.end(), {msg6008(0x0E, SUB_AI_TIMING, u32(0x01000000)), msg6008(0x16, SUB_AI_TIMING, u32(divisor)),
                       msg6008(0x0F, SUB_AI_TIMING, u32(n)), msg6008(0x13, SUB_AI_TIMING), msg6008(0x15, SUB_AI_TIMING),
                       msg6008(0x18, SUB_AI_TIMING), msg6008(0x12, SUB_AI_DATA, u32(uint32_t(2 * n * chans.size())), 0, 0),
                       msg6008(0x0F, SUB_AI_DATA), msg6008(0x09, SUB_AI_DATA), msg6008(0x09, SUB_AI_TIMING),
                       msg6008(0x0A, SUB_AI_TIMING)});
    return m;
}

const std::vector<Bytes> &aiPoll()
{
    static const std::vector<Bytes> m = {msg6008(0x13, SUB_AI_DATA), msg6008(0x10, SUB_AI_DATA)};
    return m;
}

const std::vector<Bytes> &aiTeardown()
{
    static const std::vector<Bytes> m = {msg6008(0x0B, SUB_AI_TIMING), msg6008(0x0C, SUB_AI_TIMING),
                                         msg6008(0x0C, SUB_AI_DATA), msg6008(0x0D, SUB_AI_TIMING)};
    return m;
}

Bytes dioRead(uint8_t port)
{
    return msg6008(0x0E, SUB_DIO, {0, 3, port, 0});
}

Bytes dioWrite(uint8_t port, uint8_t value)
{
    return msg6008(0x0F, SUB_DIO, {0, 3, port, 0, 3, value, 0, 0});
}

Bytes dioDirection(uint8_t p0, uint8_t p1)
{
    return msg6008(0x12, SUB_DIO, {0, 4, p0, uint8_t(p1 & 0x0F), 0, 4, 0, 0});
}

Bytes aoWrite(uint16_t chan, int16_t code)
{
    Bytes p;
    put16(p, uint16_t(code));
    put16(p, 0);
    return msg6008(0x0E, SUB_AO, p, chan);
}

Calibration Calibration::parse(const Bytes &mem)
{
    if (mem.size() < CAL_SIZE)
        throw Error("calibration memory of " + std::to_string(mem.size()) + " bytes");
    auto pair = [&](unsigned a) { return std::make_pair(double(beFloat(&mem[a])), double(beFloat(&mem[a + 4]))); };
    Calibration c;
    for (int r = 0; r < 8; r++)
        for (int ch = 0; ch < N_DIFF; ch++)
            c.diff[r][ch] = pair(CAL_DIFF + 0x20 * r + 8 * ch);
    for (int ch = 0; ch < N_RSE; ch++)
        c.rse[ch] = pair(CAL_RSE + 8 * ch);
    for (int ch = 0; ch < N_AO; ch++)
        c.ao[ch] = pair(CAL_AO + 8 * ch);
    return c;
}

} // namespace n6008

using namespace n6008;

Device6008::Device6008(Bus &bus, std::string serial) : bus_(bus), want_(std::move(serial)) {}

void Device6008::close()
{
    t_.reset();
    mem_.clear();
    cal_ = {};
    dirSet_ = false;
}

void Device6008::open()
{
    close();
    try {
        for (const UnitInfo &u : bus_.list())
            if (u.pid == PID && (want_.empty() || u.serial == want_)) {
                attach(u);
                return;
            }
        throw Error(std::string("no NI ") + MODEL + (want_.empty() ? std::string() : " with serial " + want_) +
                    " found");
    } catch (...) {
        close();  // never left open half way: no transport without its calibration
        throw;
    }
}

void Device6008::attach(const UnitInfo &u)
{
    t_ = bus_.open(u);
    info_ = u;
    // Replies and samples left by an interrupted transaction (a killed or
    // crashed IOC) would be taken for the answers to our first requests.
    for (uint8_t ep : {EP_REPLY, EP_SAMPLES})
        for (int i = 0; i < 64; i++) {
            try {
                if (t_->read(ep, 512, 100).empty())
                    break;
            } catch (const Timeout &) {
                break;
            }
        }
    setCal(readCal());
}

Bytes Device6008::send(const Bytes &m)
{
    if (!t_)
        throw TransportError(std::string("no ") + MODEL + " open");
    t_->write(EP_CMD, m, 5000);
    if (m.size() < 7 || m[6] != 1)
        return {};
    for (int stray = 0; stray < 64; stray++) {
        Bytes r = t_->read(EP_REPLY, 512, int(replyTimeout * 1000));
        if (r.size() >= 12 && r[0] == m[0] && r[1] == m[1])
            return r;
    }
    throw TransportError("no reply on channel " + std::to_string(m[1]) + " among 64");
}

Bytes Device6008::check(const Bytes &m)
{
    Bytes r = send(m);
    if (!r.empty() && r[11] != 2)
        throw Error("refused (status " + std::to_string(r[11]) + "): " + toHex(m) + " -> " + toHex(r));
    return r;
}

Bytes Device6008::readCal()
{
    Bytes mem;
    for (uint32_t a = 0; a < CAL_SIZE; a += CAL_CHUNK) {
        Bytes r = check(calRead(a));
        if (r.size() < 14 + CAL_CHUNK || (r[12] << 8 | r[13]) != int(CAL_CHUNK + 2))
            throw Error("calibration read at " + std::to_string(a) + ": " + toHex(r));
        mem.insert(mem.end(), r.begin() + 14, r.begin() + 14 + CAL_CHUNK);
    }
    return mem;
}

void Device6008::setCal(const Bytes &mem)
{
    cal_ = Calibration::parse(mem);
    mem_ = mem;
}

void Device6008::adopt(std::unique_ptr<Transport> t, const UnitInfo &u, const Bytes &mem)
{
    t_ = std::move(t);
    info_ = u;
    dirSet_ = false;
    setCal(mem);
}

/* The AI teardown: send(), not check(), so a refusal (a task not armed) is tolerated. */
void Device6008::teardown()
{
    for (const Bytes &m : aiTeardown())
        send(m);
}

static void checkChannels(const std::vector<int> &chans, double range, Term mode)
{
    if (mode == Term::RSE && std::fabs(range - RSE_RANGE) > 1e-9)
        throw Error("the 6008's RSE range is +-10 V only");
    if (mode == Term::DIFF)
        diffRangeIndex(range);
    int n = mode == Term::RSE ? N_RSE : N_DIFF;
    for (int c : chans)
        if (c < 0 || c >= n)
            throw Error("AI channel " + std::to_string(c) + " out of range for the 6008 in " +
                        (mode == Term::RSE ? "RSE" : "DIFF"));
}

double Device6008::scale(uint16_t word, double range, Term mode, int chan) const
{
    if (mode == Term::DIFF) {
        auto c = cal_.diff[diffRangeIndex(range)].at(size_t(chan));
        return c.first + (word + 0.5) * (c.second - c.first) / 65536;
    }
    auto c = cal_.rse.at(size_t(chan));
    return c.second + (int16_t(word) + 0.5) * (c.second - c.first) / 32768;
}

Bytes Device6008::samples(size_t want, double seconds, bool poll)
{
    auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    Bytes raw;
    while (raw.size() < want) {
        if (std::chrono::steady_clock::now() > deadline)  // the unit answers, the samples did not come: not a transport error
            throw Error("AI: " + std::to_string(raw.size()) + " of " + std::to_string(want) + " bytes by the deadline");
        if (poll)
            for (const Bytes &m : aiPoll())
                check(m);
        while (raw.size() < want) {
            Bytes d;
            try {
                // a whole packet each time: a shorter request overflows if the unit sends more
                d = t_->read(EP_SAMPLES, 64, poll ? POLL_MS : 3000);
            } catch (const Timeout &) {
                if (!poll)
                    throw;
                break;
            }
            if (d.empty()) {
                if (!poll)
                    throw Timeout("AI: no samples");
                break;
            }
            raw.insert(raw.end(), d.begin(), d.end());
        }
    }
    raw.resize(want);
    return raw;
}

static std::vector<uint16_t> words(const Bytes &raw)
{
    std::vector<uint16_t> w(raw.size() / 2);
    for (size_t i = 0; i < w.size(); i++)
        w[i] = uint16_t(raw[2 * i] << 8 | raw[2 * i + 1]);
    return w;
}

std::vector<uint16_t> Device6008::aiReadWords(int chan, double range, Term mode, uint32_t n)
{
    checkChannels({chan}, range, mode);
    if (n < 1 || n > 0xFFFF)
        throw Error("on-demand samples: 1..65535");
    std::vector<Bytes> m = aiOnDemand(chan, range, mode, n);
    for (size_t i = 0; i + 1 < m.size(); i++)
        check(m[i]);
    Bytes raw;
    try {
        send(m.back());
        raw = samples(size_t(2) * n, 10.0, false);
    } catch (const TransportError &) {
        throw;  // the unit is gone or wedged: no teardown to send
    } catch (...) {
        teardown();
        throw;
    }
    teardown();
    return words(raw);
}

double Device6008::aiRead(int chan, double range, Term mode, uint32_t n)
{
    std::vector<uint16_t> w = aiReadWords(chan, range, mode, n);
    double s = 0;
    for (uint16_t x : w)
        s += scale(x, range, mode, chan);
    return s / w.size();
}

uint32_t Device6008::timedDivisor(size_t nChans, double rate)
{
    if (!(rate > 0))
        throw Error("timed AI: rate must be > 0");
    double div = std::floor(CLOCK / rate + 0.5);
    if (!(div >= 1 && div <= MAX_DIVISOR))
        throw Error("timed AI: " + std::to_string(rate) + " Hz needs a clock divisor of " + std::to_string(uint64_t(div)) +
                    "; 1.." + std::to_string(MAX_DIVISOR) + " are known (366.2 Hz and up)");
    if (CLOCK / div * nChans > MAX_AI_RATE)
        throw Error("timed AI: " + std::to_string(nChans) + " channels at " + std::to_string(CLOCK / div) +
                    " Hz exceed the 6008's 10 kS/s aggregate");
    return uint32_t(div);
}

std::vector<uint16_t> Device6008::aiTimedWords(const std::vector<int> &chans, double rate, uint32_t n, double range,
                                               Term mode)
{
    checkChannels(chans, range, mode);
    if (chans.empty() || n < 1 || uint64_t(n) * 2 * chans.size() > 0xFFFFFFFFull)
        throw Error("timed AI: bad sample count " + std::to_string(n));
    uint32_t div = timedDivisor(chans.size(), rate);
    for (const Bytes &m : aiTimed(chans, range, mode, div, n))
        check(m);
    Bytes raw;
    try {
        raw = samples(size_t(2) * n * chans.size(), double(n) * div / CLOCK + 5.0, true);
    } catch (const TransportError &) {
        throw;
    } catch (...) {
        // a scan that failed (its samples never all came): stop it, refusals
        // tolerated, and drop late samples so the next read gets its own
        teardown();
        for (int i = 0; i < 64; i++) {
            try {
                if (t_->read(EP_SAMPLES, 64, 100).empty())
                    break;
            } catch (const Timeout &) {
                break;
            }
        }
        throw;
    }
    teardown();
    return words(raw);
}

int16_t Device6008::aoCode(int chan, double volts) const
{
    if (chan < 0 || chan >= N_AO)
        throw Error("the 6008 has no AO" + std::to_string(chan));
    auto c = cal_.ao[chan];
    double v = std::min(AO_MAX, std::max(0.0, volts));
    if (std::isnan(volts))
        throw Error("AO: not a number");
    double code = std::floor((v - c.first) / (c.second - c.first) * 4095 + 0.5);
    return int16_t(std::max(-2048.0, std::min(4095.0, code)));
}

int16_t Device6008::aoWrite(int chan, double volts)
{
    int16_t code = aoCode(chan, volts);
    check(n6008::aoWrite(uint16_t(chan), code));
    return code;
}

uint8_t Device6008::diRead(uint8_t port)
{
    if (port > 1)
        throw Error("the 6008 has ports 0 and 1");
    Bytes r = check(dioRead(port));
    if (r.size() < 16)
        throw Error("short DIO reply");
    return port == 0 ? r[14] : uint8_t(r[14] & 0x0F);
}

uint8_t Device6008::doWrite(uint8_t value, uint8_t mask)
{
    if (value > 0xF || mask > 0xF)
        throw Error("port 1 is 4 lines");
    uint8_t now = diRead(1);
    if (!dirSet_) {
        // as NI-DAQmx: latch the lines' state, then make them outputs, so none glitches
        check(dioWrite(1, now));
        check(dioDirection(0x00, 0x0F));
        dirSet_ = true;
    }
    uint8_t next = uint8_t(((now & ~mask) | (value & mask)) & 0x0F);
    check(dioWrite(1, next));
    return next;
}

} // namespace niusb
