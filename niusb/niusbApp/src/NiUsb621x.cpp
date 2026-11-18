#include "NiUsb621x.h"

#include <algorithm>
#include <cmath>

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

namespace niusb {

Device::Device(Bus &bus, std::string serial, const Firmware *fw) : bus_(bus), want_(std::move(serial)), fw_(fw) {}

const Model &Device::requireModel() const
{
    if (!model_)
        throw Error("no 621x open");
    return *model_;
}

const Model &Device::model() const
{
    return requireModel();
}

void Device::close()
{
    t_.reset();
    model_ = nullptr;
    eeprom_.clear();
    cal_ = {};
    aoCal_.clear();
}

namespace {
/* An exclusive flock held for a scope; a path that cannot be opened is no lock. */
struct FileLock {
    int fd = -1;
    std::string problem;
    explicit FileLock(const std::string &path)
    {
        if (path.empty())
            return;
        fd = ::open(path.c_str(), O_RDONLY);
        if (fd < 0) {
            problem = "cannot open " + path + " to lock the blank-unit load: " + std::strerror(errno);
            return;
        }
        int rc;
        while ((rc = ::flock(fd, LOCK_EX)) != 0 && errno == EINTR) {
        }
        if (rc != 0) {
            problem = "cannot lock " + path + " for the blank-unit load: " + std::strerror(errno);
            ::close(fd);
            fd = -1;
        }
    }
    ~FileLock()
    {
        if (fd >= 0) {
            ::flock(fd, LOCK_UN);
            ::close(fd);
        }
    }
};
} // namespace

void Device::open()
{
    close();
    fresh_ = false;
    lockProblem.clear();
    try {
        openUnlocked();
    } catch (...) {
        close();  // never left open half way: no transport without its calibration
        throw;
    }
}

void Device::openUnlocked()
{
    std::vector<UnitInfo> units = bus_.list();
    std::vector<UnitInfo> blanks;  // places seen blank in this open
    auto noteBlanks = [&] {
        for (const UnitInfo &u : units)
            if (u.pid == PID_BLANK)
                blanks.push_back(u);
    };
    auto wasBlank = [&](const UnitInfo &u) {
        for (const UnitInfo &b : blanks)
            if (b.samePlace(u))
                return true;
        return false;
    };
    noteBlanks();
    for (const UnitInfo &u : units)
        if (modelByPid(u.pid) && (want_.empty() || u.serial == want_)) {
            attach(u, false);
            return;
        }
    bool anyBlank = false;
    for (const UnitInfo &u : units)
        anyBlank = anyBlank || u.pid == PID_BLANK;
    if (!anyBlank)
        throw Error("no NI USB-621x" + (want_.empty() ? std::string() : " with serial " + want_) + " found");
    // Load blank units under the host-wide lock, listing again once held:
    // another IOC may have loaded one meanwhile (perhaps ours).
    FileLock lock(loadLock);
    lockProblem = lock.problem;
    units = bus_.list();
    noteBlanks();
    for (const UnitInfo &u : units)
        if (modelByPid(u.pid) && (want_.empty() || u.serial == want_)) {
            attach(u, false);  // another IOC loaded it while this one waited
            fresh_ = wasBlank(u);
            return;
        }
    std::string seen;
    for (const UnitInfo &u : units) {
        if (u.pid != PID_BLANK)
            continue;
        if (!fw_)
            throw Error("a blank 621x is on the bus and no NI firmware is loaded");
        loadFx2(u);
        UnitInfo l = waitLoaded(u);
        attach(l, true);
        if (want_.empty() || l.serial == want_) {
            fresh_ = true;
            return;
        }
        seen += " " + l.serial;
        close();  // another IOC's unit: it stays loaded and brought up for it
    }
    throw Error("no NI USB-621x" + (want_.empty() ? std::string() : " with serial " + want_) + " found" +
                (seen.empty() ? std::string() : "; loaded" + seen + " on the way"));
}

void Device::attach(const UnitInfo &u, bool fresh)
{
    t_ = bus_.open(u);
    info_ = u;
    model_ = modelByPid(u.pid);
    if (fresh) {
        bringup();
        return;
    }
    try {
        readEeprom();
    } catch (const TransportError &) {
        throw;
    } catch (const Error &) {
        // Firmware loaded but the FPGAs not (an earlier load stopped half
        // way): the EEPROM read is refused, so bring the FPGAs up now.
        if (!fw_)
            throw;
        bringup();
    }
}

void Device::fx2Write(Transport &t, uint8_t req, uint16_t base, const Bytes &data)
{
    for (size_t off = 0; off < data.size(); off += 512) {
        Bytes c(data.begin() + off, data.begin() + std::min(data.size(), off + 512));
        t.ctrlOut(req, uint16_t(base + off), 0, c, 2000);
        if (t.ctrlIn(req, uint16_t(base + off), 0, uint16_t(c.size()), 2000) != c)
            throw Error("FX2 readback mismatch, request " + std::to_string(req) + " at " + std::to_string(base + off));
    }
}

void Device::loadFx2(const UnitInfo &u)
{
    std::unique_ptr<Transport> t = bus_.open(u);
    auto cpucs = [&](uint8_t hold) { t->ctrlOut(0xA0, 0xE600, 0, Bytes{hold}, 2000); };
    cpucs(1);
    fx2Write(*t, 0xA0, 0, fw_->loader);
    cpucs(0);
    bus_.sleep(0.05);
    t->ctrlIn(0xB0, 0, 0, 2, 2000);  // the loader's version (70 72), as NI-DAQmx asks
    fx2Write(*t, 0xA4, 0x4000, fw_->bank0);
    fx2Write(*t, 0xA5, 0x4000, fw_->bank1);
    cpucs(1);
    fx2Write(*t, 0xA0, 0, fw_->main16k);
    cpucs(0);
}

UnitInfo Device::waitLoaded(const UnitInfo &blank)
{
    for (int i = 0; i < 100; i++) {
        bus_.sleep(0.2);
        for (const UnitInfo &u : bus_.list())
            if (u.samePlace(blank) && modelByPid(u.pid)) {
                bus_.sleep(1.0);
                return u;
            }
    }
    throw Error("the 621x did not come back with a known product ID after its firmware load");
}

Bytes Device::send(const Bytes &m)
{
    if (!t_)
        throw TransportError("no 621x open");
    t_->write(EP_CMD, m, 15000);  // 08 03 ff (the Xilinx) blocks ~3.3 s
    if (m.size() < 7 || m[6] != 1)
        return {};
    for (int stray = 0; stray < 64; stray++) {
        Bytes r = t_->read(EP_REPLY, 512, int(replyTimeout * 1000));
        if (r.size() >= 12 && r[0] == m[0] && r[1] == m[1])
            return r;
    }
    throw TransportError("no reply on channel " + std::to_string(m[1]) + " among 64");
}

Bytes Device::check(const Bytes &m)
{
    Bytes r = send(m);
    if (!r.empty() && r[11] != 2)
        throw Error("refused (status " + std::to_string(r[11]) + "): " + toHex(m) + " -> " + toHex(r));
    return r;
}

void Device::fpga(uint8_t target, const Bytes &image)
{
    if (target == 0xFF)  // the length, announced as NI-DAQmx does before the Xilinx
        send(announce(0, 0x0002, uint32_t(image.size())));
    check(msg(0, 0x08, target));
    for (size_t off = 0; off < image.size(); off += 512)
        t_->write(EP_FPGA, Bytes(image.begin() + off, image.begin() + std::min(image.size(), off + 512)), 5000);
    for (int i = 0; i < 100; i++) {
        Bytes r = check(msg(0, 0x09, target));
        if (r.size() == 12 || r[12] == 1)  // FPGA0 answers a data word, the Xilinx none
            return;
        bus_.sleep(0.05);
    }
    throw Error("FPGA " + std::to_string(target) + " never reported done");
}

void Device::bringup()
{
    if (!fw_)
        throw Error("the 621x needs its FPGAs loaded and no NI firmware is loaded");
    for (const Bytes &m : bringupHead())
        send(m);
    fpga(0xFE, fw_->fpga0);
    fpga(0xFF, fw_->fpga1);
    for (const Bytes &m : postFpgaHead())
        send(m);
    readEeprom();
    for (const Bytes &m : postFpgaTail())
        check(m);
}

void Device::readEeprom()
{
    check(eepromOpen());
    Bytes out;
    for (uint32_t off = 0; off < EEPROM_SIZE; off += EEPROM_CHUNK) {
        uint32_t n = std::min(EEPROM_CHUNK, EEPROM_SIZE - off);
        Bytes r = check(eepromRead(off, n));
        if (r.size() < 14 + n)
            throw Error("short EEPROM reply at " + std::to_string(off));
        out.insert(out.end(), r.begin() + 14, r.begin() + 14 + n);
    }
    setEeprom(out);
}

void Device::adopt(std::unique_ptr<Transport> t, const UnitInfo &u, const Bytes &eeprom)
{
    t_ = std::move(t);
    info_ = u;
    model_ = modelByPid(u.pid);
    setEeprom(eeprom);
}

void Device::setEeprom(const Bytes &eeprom)
{
    if (eeprom.size() < EEPROM_SIZE)
        throw Error("EEPROM of " + std::to_string(eeprom.size()) + " bytes");
    eeprom_ = eeprom;
    for (int i = 0; i < 7; i++)
        cal_[i] = {beFloat(&eeprom_[SELF_CAL_TABLE + 8 * i]), beFloat(&eeprom_[SELF_CAL_TABLE + 8 * i + 4])};
    aoCal_.clear();
    for (unsigned a : requireModel().aoCal)
        aoCal_.push_back({beFloat(&eeprom_[a]), beFloat(&eeprom_[a + 4])});
}

std::pair<double, double> Device::aiCal(double range) const
{
    return cal_[calSlot(range)];
}

double Device::scale(int16_t code, double range) const
{
    auto c = aiCal(range);
    return c.first * code + c.second;
}

void Device::teardown(bool timed)
{
    for (const Bytes &m : aiTeardown(timed))
        send(m);
}

static void checkChannels(const Model &m, const std::vector<int> &chans, Mode mode)
{
    int n = mode == Mode::DIFF ? m.nAi / 2 : m.nAi;
    for (int c : chans)
        if (c < 0 || c >= n)
            throw Error("AI channel " + std::to_string(c) + " out of range for the " + m.name +
                        (mode == Mode::DIFF ? " in DIFF" : ""));
}

std::vector<int16_t> Device::aiReadCodes(int chan, double range, Mode mode, int n)
{
    checkChannels(requireModel(), {chan}, mode);
    gainCode(range);
    for (const Bytes &m : aiSetup({chan}, range, mode))
        check(m);
    std::vector<int16_t> codes;
    try {
        for (int i = 0; i < n; i++) {
            for (const Bytes &m : aiSample())
                check(m);
            Bytes d = t_->read(EP_DATA, 512, 3000);
            if (d.size() != 2)
                throw Error("AI sample of " + std::to_string(d.size()) + " bytes");
            codes.push_back(int16_t(d[0] | d[1] << 8));
        }
    } catch (const TransportError &) {
        throw;  // the unit is gone or wedged: no teardown to send
    } catch (...) {
        teardown(false);
        throw;
    }
    teardown(false);
    return codes;
}

double Device::aiRead(int chan, double range, Mode mode, int n)
{
    std::vector<int16_t> c = aiReadCodes(chan, range, mode, n);
    double s = 0;
    for (int16_t x : c)
        s += scale(x, range);
    return s / c.size();
}

double Device::aiTimedRate(double rate) const
{
    return TIMEBASE / std::round(TIMEBASE / rate);
}

std::vector<int16_t> Device::aiTimedCodes(const std::vector<int> &chans, double rate, uint32_t n, double range, Mode mode)
{
    checkChannels(requireModel(), chans, mode);
    gainCode(range);
    double div = std::round(TIMEBASE / rate);
    if (n < 1 || !(div >= 1) || div > 0xFFFFFFFF)
        throw Error("timed AI: bad timing, rate " + std::to_string(rate) + " Hz, " + std::to_string(n) + " samples");
    if (TIMEBASE / div * chans.size() > MAX_AI_RATE)
        throw Error("timed AI: " + std::to_string(chans.size()) + " channels at " + std::to_string(TIMEBASE / div) +
                    " Hz exceed the 621x's 250 kS/s aggregate");
    for (const Bytes &m : aiSetup(chans, range, mode, uint32_t(div), n))
        check(m);
    size_t want = size_t(2) * n * chans.size();
    Bytes raw;
    try {
        check(aiStart());
        double seconds = n * div / TIMEBASE + 5.0;
        int ms = int(seconds * 1000);
        while (raw.size() < want) {
            Bytes d = t_->read(EP_DATA, std::min<size_t>(want - raw.size(), 1 << 16), ms);
            if (d.empty())
                throw Timeout("timed AI: " + std::to_string(raw.size()) + " of " + std::to_string(want) + " bytes");
            raw.insert(raw.end(), d.begin(), d.end());
        }
    } catch (const TransportError &) {
        throw;
    } catch (...) {
        teardown(true);
        throw;
    }
    teardown(true);
    std::vector<int16_t> codes(want / 2);
    for (size_t i = 0; i < codes.size(); i++)
        codes[i] = int16_t(raw[2 * i] | raw[2 * i + 1] << 8);
    return codes;
}

int16_t Device::aoWrite(int chan, double volts)
{
    const Model &m = requireModel();
    if (chan < 0 || size_t(chan) >= aoCal_.size())
        throw Error(std::string("the ") + m.name + " has no AO" + std::to_string(chan));
    if (!(volts >= -10.0 && volts <= 10.0))
        throw Error("AO range is +-10 V");
    double c = std::round(aoCal_[chan].first * volts + aoCal_[chan].second);
    int16_t code = int16_t(std::max(-32768.0, std::min(32767.0, c)));
    for (const Bytes &x : niusb::aoWrite(uint16_t(chan), code))
        check(x);
    return code;
}

uint32_t Device::pfiRead()
{
    Bytes r = check(niusb::pfiRead());
    if (r.size() < 16)
        throw Error("short PFI reply");
    return be32(&r[12]);
}

void Device::doWrite(uint8_t value)
{
    if (value > 0xF)
        throw Error("port 1 is 4 lines");
    for (const Bytes &m : niusb::doWrite(value))
        check(m);
}

} // namespace niusb
