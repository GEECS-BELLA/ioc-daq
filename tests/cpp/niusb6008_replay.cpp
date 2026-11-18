/* Replays a NI-DAQmx capture of a USB-6008 against the driver: the script
 * (from tests/replay6008.py) queues the unit's replies and samples, each due
 * once as many ep01 messages have been sent as preceded it in the capture,
 * and names the calls; every message the driver sends is printed, for the
 * test to compare with what NI-DAQmx sent.
 *
 * script lines:
 *   cal <hex>                  adopt an open unit with this calibration memory (no I/O)
 *   ep81|ep82 <after> <hex>    a reply or sample transfer, due after <after> messages
 *   open                       find the unit and read its calibration
 *   ai <chan> <range> <RSE|DIFF> <n>
 *   timed <c,c,...> <rate> <n> <range> <mode>
 *   ao <chan> <volts> | do <value> | di <port>
 * output lines: ep01 <hex>, result <...>, error <what>. */
#include <deque>
#include <fstream>
#include <iostream>
#include <sstream>

#include "NiUsb6008.h"

using namespace niusb;
using namespace niusb::n6008;

namespace {

struct Due {
    size_t after;
    Bytes data;
};

struct Replay {
    std::deque<Due> ep81, ep82;
    size_t writes = 0;
};

class ReplayTransport : public Transport {
public:
    explicit ReplayTransport(Replay &r) : r_(r) {}
    void ctrlOut(uint8_t, uint16_t, uint16_t, const Bytes &, int) override { throw Error("replay: control out"); }
    Bytes ctrlIn(uint8_t, uint16_t, uint16_t, uint16_t, int) override { throw Error("replay: control in"); }
    void write(uint8_t ep, const Bytes &data, int) override
    {
        std::cout << "ep" << (ep < 16 ? "0" : "") << std::hex << int(ep) << std::dec << " " << toHex(data) << "\n";
        r_.writes++;
    }
    Bytes read(uint8_t ep, size_t maxLen, int) override
    {
        std::deque<Due> &q = ep == EP_REPLY ? r_.ep81 : r_.ep82;
        if (q.empty() || q.front().after > r_.writes)
            throw Timeout("replay: nothing due on ep " + std::to_string(ep));
        Bytes &x = q.front().data;
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
        u.pid = PID;
        u.serial = "REPLAY";
        return {u};
    }
    std::unique_ptr<Transport> open(const UnitInfo &) override { return std::make_unique<ReplayTransport>(r_); }
    void sleep(double) override {}

private:
    Replay &r_;
};

Term mode(const std::string &s)
{
    if (s == "RSE")
        return Term::RSE;
    if (s == "DIFF")
        return Term::DIFF;
    throw Error("mode " + s);
}

} // namespace

int main(int argc, char **argv)
{
    if (argc != 2) {
        std::cerr << "usage: niusb6008_replay SCRIPT\n";
        return 2;
    }
    std::ifstream in(argv[1]);
    Replay r;
    ReplayBus bus(r);
    Device6008 dev(bus, "");
    std::string line;
    std::cout.precision(12);
    while (std::getline(in, line)) {
        std::istringstream ls(line);
        std::string op;
        ls >> op;
        try {
            if (op.empty() || op[0] == '#') {
                continue;
            } else if (op == "cal") {
                std::string h;
                ls >> h;
                UnitInfo u;
                u.pid = PID;
                u.serial = "REPLAY";
                dev.adopt(std::make_unique<ReplayTransport>(r), u, fromHex(h));
            } else if (op == "ep81" || op == "ep82") {
                size_t after;
                std::string h;
                ls >> after >> h;
                (op == "ep81" ? r.ep81 : r.ep82).push_back({after, fromHex(h)});
            } else if (op == "open") {
                dev.open();
                std::cout << "result open " << toHex(dev.calMemory()) << "\n";
            } else if (op == "ai") {
                int ch;
                uint32_t n;
                double range;
                std::string m;
                ls >> ch >> range >> m >> n;
                std::vector<uint16_t> w = dev.aiReadWords(ch, range, mode(m), n);
                std::cout << "result ai";
                for (uint16_t x : w)
                    std::cout << " " << dev.scale(x, range, mode(m), ch);
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
                std::vector<uint16_t> w = dev.aiTimedWords(chans, rate, n, range, mode(m));
                std::cout << "result timed " << w.size();
                for (size_t i = 0; i < std::min<size_t>(w.size(), 6); i++)
                    std::cout << " " << dev.scale(w[i], range, mode(m), chans[i % chans.size()]);
                std::cout << "\n";
            } else if (op == "ao") {
                int ch;
                double v;
                ls >> ch >> v;
                int16_t code = dev.aoWrite(ch, v);
                std::cout << "result ao " << code << "\n";
            } else if (op == "do") {
                int v;
                ls >> v;
                int now = dev.doWrite(uint8_t(v));  // its requests print first
                std::cout << "result do " << now << "\n";
            } else if (op == "di") {
                int p;
                ls >> p;
                int v = dev.diRead(uint8_t(p));
                std::cout << "result di " << v << "\n";
            } else {
                throw Error("unknown script line: " + line);
            }
        } catch (const std::exception &e) {
            std::cout << "error " << e.what() << "\n";
        }
    }
    return 0;
}
