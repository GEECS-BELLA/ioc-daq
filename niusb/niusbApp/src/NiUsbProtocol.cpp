#include "NiUsbProtocol.h"

#include <cmath>
#include <cstring>

namespace niusb {

const std::vector<Model> &models()
{
    // AO calibration addresses: AO0 at 0x22D and AO1 at 0x25E, measured on a
    // USB-6211 against a loopback (README "NI USB-621x").
    static const std::vector<Model> m = {
        {0x7270, "USB-6211", 16, {0x22D, 0x25E}, 4, 4},
        {0x726F, "USB-6210", 16, {}, 4, 4},
    };
    return m;
}

const Model *modelByPid(uint16_t pid)
{
    for (const Model &m : models())
        if (m.pid == pid)
            return &m;
    return nullptr;
}

const std::vector<double> &ranges()
{
    static const std::vector<double> r = {10.0, 5.0, 1.0, 0.2};
    return r;
}

static int rangeIndex(double range)
{
    const std::vector<double> &r = ranges();
    for (size_t i = 0; i < r.size(); i++)
        if (std::fabs(r[i] - range) < 1e-9)
            return int(i);
    throw Error("the 621x ranges are +-10, 5, 1 and 0.2 V, not " + std::to_string(range));
}

int gainCode(double range)
{
    static const int g[] = {0, 1, 4, 5};
    return g[rangeIndex(range)];
}

int calSlot(double range)
{
    static const int s[] = {0, 1, 3, 5};
    return s[rangeIndex(range)];
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

static Bytes frame(uint16_t ch, const Bytes &body)
{
    Bytes out;
    uint16_t n = uint16_t(body.size() + 6);
    put16(out, ch);
    put16(out, n);
    put16(out, uint16_t(n - 4));
    out.insert(out.end(), body.begin(), body.end());
    return out;
}

Bytes msg(uint16_t ch, uint8_t verb, uint8_t subsys, uint16_t index, const Bytes &payload, uint8_t kind)
{
    Bytes body = {kind, verb, 0x03, subsys};
    put16(body, index);
    body.insert(body.end(), payload.begin(), payload.end());
    return frame(ch, body);
}

Bytes announce(uint16_t ch, uint16_t pipe, uint32_t nbytes)
{
    Bytes body = {0x00, 0x08, 0x00, 0x02};
    put16(body, pipe);
    put32(body, nbytes);
    return frame(ch, body);
}

Bytes fromHex(const std::string &hex)
{
    Bytes out;
    for (size_t i = 0; i + 1 < hex.size(); i += 2)
        out.push_back(uint8_t(std::stoul(hex.substr(i, 2), nullptr, 16)));
    return out;
}

std::string toHex(const Bytes &b)
{
    static const char *d = "0123456789abcdef";
    std::string s;
    for (uint8_t x : b) {
        s += d[x >> 4];
        s += d[x & 15];
    }
    return s;
}

uint32_t be32(const uint8_t *p)
{
    return uint32_t(p[0]) << 24 | uint32_t(p[1]) << 16 | uint32_t(p[2]) << 8 | p[3];
}

float beFloat(const uint8_t *p)
{
    uint32_t u = be32(p);
    float f;
    std::memcpy(&f, &u, 4);
    return f;
}

Bytes aiConfig(const std::vector<int> &chans, double range, Mode mode)
{
    if (chans.empty())
        throw Error("an AI channel list needs a channel");
    Bytes words;
    for (size_t i = 0; i < chans.size(); i++) {
        uint16_t w = uint16_t(gainCode(range) << 9 | int(mode) << 6 | (chans[i] & 0xF));
        if (i + 1 == chans.size())
            w |= 0x4000;
        put16(words, w);
    }
    Bytes out;
    put16(out, uint16_t(2 + words.size()));
    out.insert(out.end(), words.begin(), words.end());
    while (out.size() % 4)
        out.push_back(0);
    return out;
}

static const Bytes AI_ONDEMAND_ROUTE = fromHex("00010000000112000101130000011c000001010000000001010000000101000001000000");
static const Bytes AI_TIMED_ROUTE = fromHex("00010000000100000101130000011c000000010000000001010100000101000000000100");

std::vector<Bytes> aiSetup(const std::vector<int> &chans, double range, Mode mode, uint32_t divisor, uint32_t n)
{
    Bytes timing;
    put32(timing, 3);
    put32(timing, 0x118);
    if (divisor) {
        put32(timing, 2);
        put32(timing, divisor);
        put32(timing, n - 1);
    } else {
        put32(timing, 0);
        put32(timing, 0);
        put32(timing, 0xFFFFFFFF);
    }
    put32(timing, 0xFFFFFFFF);
    std::vector<Bytes> m = {
        msg(1, 0x0A, 0x02, 0, fromHex("00320000")),
        msg(1, 0x08, 0x15, 0, fromHex("0000000008000000")),
        msg(1, 0x0B, 0x12),
        msg(1, 0x0A, 0x12, 0, aiConfig(chans, range, mode)),
        msg(1, 0x0C, 0x12),
        msg(1, 0x10, 0x13, 0, timing),
        msg(1, 0x11, 0x13, 0, divisor ? AI_TIMED_ROUTE : AI_ONDEMAND_ROUTE),
        msg(1, 0x08, 0x10),
        msg(1, 0x09, 0x11, 0, fromHex("00000002")),
    };
    if (divisor)  // the whole acquisition announced on pipe 8, then armed
        m.push_back(announce(7, 0x0008, uint32_t(2 * n * chans.size())));
    m.push_back(msg(1, 0x0D, 0x11));
    m.push_back(msg(1, 0x0A, 0x13));
    if (divisor)
        m.push_back(msg(0, 0x04, 0x15, 0, fromHex("00040001")));
    m.push_back(msg(1, 0x0C, 0x15));
    m.push_back(msg(1, 0x0A, 0x11));
    m.push_back(msg(1, 0x09, 0x10));
    m.push_back(msg(1, 0x0B, 0x13, 0, fromHex("00000000")));
    return m;
}

const std::vector<Bytes> &aiSample()
{
    static const std::vector<Bytes> m = {announce(1, 0x0008, 2), msg(1, 0x0C, 0x13)};
    return m;
}

Bytes aiStart()
{
    return msg(1, 0x0C, 0x13);
}

const std::vector<Bytes> &aiTeardown(bool timed)
{
    static const uint8_t vs[][2] = {{0x0D, 0x13}, {0x0E, 0x13}, {0x0A, 0x10}, {0x0B, 0x11}, {0x0D, 0x15},
                                    {0x0C, 0x11}, {0x0B, 0x10}, {0x0F, 0x13}, {0x09, 0x15}};
    static const std::vector<Bytes> plain = [] {
        std::vector<Bytes> m;
        for (auto &v : vs)
            m.push_back(msg(1, v[0], v[1]));
        return m;
    }();
    static const std::vector<Bytes> withTimed = [] {
        std::vector<Bytes> m(plain.begin(), plain.begin() + 5);
        m.push_back(msg(0, 0x05, 0x15, 0, fromHex("00040001")));
        m.insert(m.end(), plain.begin() + 5, plain.end());
        return m;
    }();
    return timed ? withTimed : plain;
}

static std::vector<Bytes> hexList(std::initializer_list<const char *> l)
{
    std::vector<Bytes> m;
    for (const char *h : l)
        m.push_back(fromHex(h));
    return m;
}

const std::vector<Bytes> &bringupHead()
{
    static const std::vector<Bytes> m = hexList(
        {"0000000c0008010000000000", "0000000c0008010200000000", "0000000c0008010000000000"});
    return m;
}

const std::vector<Bytes> &postFpgaHead()
{
    static const std::vector<Bytes> m = hexList({"0000000c0008010000000000", "0000000c0008010904010000"});
    return m;
}

const std::vector<Bytes> &postFpgaTail()
{
    static const std::vector<Bytes> m = hexList({
        "00010010000c01080316000003010301", "0001000c0008010803130000",
        "0001000c0008010803110000", "0001001400100108032000000001000000000000",
        "00010010000c010a0320000000040000", "00010010000c010a0320000000040000",
        "0001001400100108032000010001000000000000", "00010010000c010a0320000100040000",
        "00010010000c010a0320000100040000", "00010010000c01080321000001000001",
        "00010010000c010b0320000000000000", "0001001800140118032e0000000000000000000000000000",
        "00010010000c010e032e000000000000", "0001001800140118032e0000000000010000000000000000",
        "00010010000c010e032e000000000001", "000100140010010a032800030000f0f001000000",
        "00010010000c010a0302000002280000", "00010010000c010a03020000022f0000",
        "00010010000c010a03020000015f0000", "00010010000c010a03020000015e0000",
        "00010010000c010a03020000015d0000", "00010010000c010a03020000015c0000",
        "00010010000c010a03020000015b0000", "00010010000c010a03020000015a0000",
        "00010010000c010a0302000001590000", "00010010000c010a0302000001580000",
        "00010010000c010a0302000000340000", "00010010000c010a0302000000b40100",
    });
    return m;
}

Bytes eepromOpen()
{
    return msg(1, 0x0C, 0x05);
}

Bytes eepromRead(uint32_t offset, uint32_t n)
{
    Bytes p;
    put32(p, offset);
    put32(p, n);
    return msg(1, 0x08, 0x05, 0, p);
}

std::vector<Bytes> aoWrite(uint16_t chan, int16_t code)
{
    Bytes v;
    put16(v, 0x0004);
    put16(v, uint16_t(code));
    return {msg(1, 0x08, 0x20, chan, fromHex("0001000000000000")), msg(1, 0x0A, 0x20, chan, v),
            msg(1, 0x09, 0x20, chan)};
}

Bytes pfiRead()
{
    return msg(1, 0x08, 0x28, 2);
}

std::vector<Bytes> doWrite(uint8_t value)
{
    Bytes p = fromHex("000000f0");
    put32(p, uint32_t(value & 0xF) << DO_SHIFT);
    return {msg(1, 0x0A, 0x28, 3, fromHex("000000f001000000")), msg(1, 0x09, 0x28, 3, p)};
}

} // namespace niusb
