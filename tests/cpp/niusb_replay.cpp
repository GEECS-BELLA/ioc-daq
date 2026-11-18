/* Replays a NI-DAQmx capture against the driver: the script (from
 * tests/replay.py) queues the unit's replies and names the calls; every
 * request the driver makes is printed, for the test to compare with what
 * NI-DAQmx sent.
 *
 * script lines:
 *   fw <loader> <main16k> <bank0> <bank1> <fpga0> <fpga1>   firmware of these sizes (filler bytes)
 *   eeprom <hex>          adopt an open unit with this EEPROM (no I/O)
 *   ep81 <hex> | ep88 <hex>   a reply or data transfer, in order
 *   open                  find, load and bring up a blank unit
 *   ai <chan> <range> <RSE|NRSE|DIFF> <n>
 *   timed <c,c,...> <rate> <n> <range> <mode>
 *   ao <chan> <volts> | do <value> | pfi
 * output lines: ctrl out|in <req> <value> <index> <length>, ep01 <hex>,
 * ep06 <bytes> (one stream), result <...>, error <what>. */
#include <deque>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>

#include "NiUsb621x.h"

using namespace niusb;

namespace {

struct Replay {
    std::deque<Bytes> ep81, ep88;
    std::map<uint32_t, Bytes> fx2;     // req << 16 | value -> last written
    bool loaded = false, banks = false, main = false;
    uint8_t cpucs = 0;
    size_t streamed = 0;               // the ep06 stream being written
    void flush()
    {
        if (streamed)
            std::cout << "ep06 " << streamed << "\n";
        streamed = 0;
    }
};

class ReplayTransport : public Transport {
public:
    explicit ReplayTransport(Replay &r) : r_(r) {}
    void ctrlOut(uint8_t req, uint16_t value, uint16_t index, const Bytes &data, int) override
    {
        r_.flush();
        std::cout << "ctrl out " << int(req) << " " << value << " " << index << " " << data.size() << "\n";
        if (req == 0xA0 && value == 0xE600) {
            r_.cpucs = data.at(0);
            if (!r_.cpucs && r_.main)
                r_.loaded = true;
            return;
        }
        r_.fx2[uint32_t(req) << 16 | value] = data;
        if (req == 0xA4 || req == 0xA5)
            r_.banks = true;
        if (req == 0xA0 && r_.banks && r_.cpucs)
            r_.main = true;
    }
    Bytes ctrlIn(uint8_t req, uint16_t value, uint16_t index, uint16_t length, int) override
    {
        r_.flush();
        std::cout << "ctrl in " << int(req) << " " << value << " " << index << " " << length << "\n";
        if (req == 0xB0)
            return {0x70, 0x72};
        auto it = r_.fx2.find(uint32_t(req) << 16 | value);
        return it == r_.fx2.end() ? Bytes(length, 0) : it->second;
    }
    void write(uint8_t ep, const Bytes &data, int) override
    {
        if (ep == EP_FPGA) {
            r_.streamed += data.size();
            return;
        }
        r_.flush();
        std::cout << "ep" << (ep < 16 ? "0" : "") << std::hex << int(ep) << std::dec << " " << toHex(data) << "\n";
    }
    Bytes read(uint8_t ep, size_t maxLen, int) override
    {
        r_.flush();
        std::deque<Bytes> &q = ep == EP_REPLY ? r_.ep81 : r_.ep88;
        if (q.empty())
            throw Timeout("replay: nothing left on ep " + std::to_string(ep));
        Bytes &x = q.front();
        Bytes out(x.begin(), x.begin() + std::min(maxLen, x.size()));
        x.erase(x.begin(), x.begin() + out.size());
        if (x.empty())
            q.pop_front();
        return out;
    }

private:
    Replay &r_;
};

class ReplayBus : public Bus {
public:
    explicit ReplayBus(Replay &r) : r_(r) {}
    std::vector<UnitInfo> list() override
    {
        UnitInfo u;
        u.bus = 1;
        u.ports = {1};
        u.pid = r_.loaded ? 0x7270 : PID_BLANK;
        u.serial = r_.loaded ? "REPLAY" : "";
        return {u};
    }
    std::unique_ptr<Transport> open(const UnitInfo &) override { return std::make_unique<ReplayTransport>(r_); }
    void sleep(double) override {}

private:
    Replay &r_;
};

Mode mode(const std::string &s)
{
    if (s == "RSE")
        return Mode::RSE;
    if (s == "NRSE")
        return Mode::NRSE;
    if (s == "DIFF")
        return Mode::DIFF;
    throw Error("mode " + s);
}

} // namespace

int main(int argc, char **argv)
{
    if (argc != 2) {
        std::cerr << "usage: niusb_replay SCRIPT\n";
        return 2;
    }
    std::ifstream in(argv[1]);
    Replay r;
    ReplayBus bus(r);
    Firmware fw;
    Device dev(bus, "", &fw);
    std::string line;
    std::cout.precision(9);
    while (std::getline(in, line)) {
        std::istringstream ls(line);
        std::string op;
        ls >> op;
        try {
            if (op.empty() || op[0] == '#') {
                continue;
            } else if (op == "fw") {
                size_t n[6];
                for (size_t &x : n)
                    ls >> x;
                Bytes *dst[6] = {&fw.loader, &fw.main16k, &fw.bank0, &fw.bank1, &fw.fpga0, &fw.fpga1};
                for (int i = 0; i < 6; i++) {
                    dst[i]->resize(n[i]);
                    for (size_t j = 0; j < n[i]; j++)
                        (*dst[i])[j] = uint8_t(j * 7 + i);
                }
            } else if (op == "eeprom") {
                std::string h;
                ls >> h;
                UnitInfo u;
                u.pid = 0x7270;
                u.serial = "REPLAY";
                dev.adopt(std::make_unique<ReplayTransport>(r), u, fromHex(h));
            } else if (op == "ep81" || op == "ep88") {
                std::string h;
                ls >> h;
                (op == "ep81" ? r.ep81 : r.ep88).push_back(fromHex(h));
            } else if (op == "open") {
                dev.open();
                r.flush();
                std::cout << "result open " << dev.model().name << " " << dev.serial() << "\n";
            } else if (op == "ai") {
                int ch, n;
                double range;
                std::string m;
                ls >> ch >> range >> m >> n;
                std::vector<int16_t> c = dev.aiReadCodes(ch, range, mode(m), n);
                std::cout << "result ai";
                for (int16_t x : c)
                    std::cout << " " << dev.scale(x, range);
                std::cout << "\n";
            } else if (op == "timed") {
                std::string cs, m;
                double rate, range;
                uint32_t n;
                ls >> cs >> rate >> n >> range >> m;
                std::vector<int> chans;
                std::stringstream ss(cs);
                for (std::string c; std::getline(ss, c, ',');)
                    chans.push_back(std::stoi(c));
                std::vector<int16_t> c = dev.aiTimedCodes(chans, rate, n, range, mode(m));
                std::cout << "result timed " << c.size();
                for (size_t i = 0; i < std::min<size_t>(c.size(), 6); i++)
                    std::cout << " " << dev.scale(c[i], range);
                std::cout << "\n";
            } else if (op == "ao") {
                int ch;
                double v;
                ls >> ch >> v;
                int16_t code = dev.aoWrite(ch, v);  // its requests print first
                std::cout << "result ao " << code << "\n";
            } else if (op == "do") {
                int v;
                ls >> v;
                dev.doWrite(uint8_t(v));
                std::cout << "result do\n";
            } else if (op == "pfi") {
                uint32_t w = dev.pfiRead();
                std::cout << "result pfi " << w << "\n";
            } else {
                throw Error("unknown script line: " + line);
            }
        } catch (const std::exception &e) {
            r.flush();
            std::cout << "error " << e.what() << "\n";
        }
    }
    r.flush();
    return 0;
}
