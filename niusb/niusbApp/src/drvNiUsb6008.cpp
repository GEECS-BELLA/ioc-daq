/* drvNiUsb6008: one NI USB-6008 as an asyn port (ASYN_CANBLOCK: the port's
 * thread makes every USB call, one at a time). The 621x's parameter names
 * (drvNiUsb621x.cpp), so GEECS sees one DAQ family; what differs:
 *
 *   AI_RANGE   int32, addr   0..7: +-20, 10, 5, 4, 2.5, 2, 1.25, 1 V (DIFF);
 *                            RSE is +-10 V (1) only
 *   AI_MODE    int32, addr   0 RSE (AI0-7), 2 DIFF (AI0-3, each against AI n+4);
 *                            1 (NRSE) is refused
 *   AO_VALUE   float64, addr AO0, AO1, 0..5 V (clamped), from the unit's calibration
 *   DI         uint32Digital P0.0-7
 *   DO, DO_RBV uint32Digital P1.0-3; a write reads P1 first and keeps the
 *                            lines outside its mask as they read
 *   WD_*       as the 621x, on the 24 MHz clock (divisor 1..65535), 10 kS/s
 *              aggregate at most; WD_RANGE / WD_MODE as AI_RANGE / AI_MODE
 *   FIRMWARE   octet: none needed
 *
 * The open reads the unit's calibration memory and nothing else: no AO and
 * no DIO direction, so an IOC restart leaves the outputs as they are. A
 * transport error (the unit unplugged, a USB error) closes the unit; the next
 * request reopens it, at most once per RETRY_S. The libusb bus makes a new
 * context for each scan while no unit is open (NiUsbLibusb.cpp). */
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <asynPortDriver.h>
#include <epicsExport.h>
#include <epicsStdio.h>
#include <epicsTime.h>
#include <errlog.h>
#include <iocsh.h>

#include "NiUsb6008.h"
#include "NiUsb6008Fake.h"
#include "NiUsbLibusb.h"

using namespace niusb;
using namespace niusb::n6008;

static const double RETRY_S = 5.0;
static const double MAX_SCAN_S = 30.0;   // a scan holds the port thread; the other records wait
static const int MAX_CHANNELS = 8;
static const int RANGE_10V = 1;          // the index of +-10 V in diffRanges(): RSE's only range

class drvNiUsb6008 : public asynPortDriver {
public:
    drvNiUsb6008(const char *port, const char *serial, int simulate, int maxPoints);

    asynStatus readFloat64(asynUser *u, epicsFloat64 *value) override;
    asynStatus writeFloat64(asynUser *u, epicsFloat64 value) override;
    asynStatus writeInt32(asynUser *u, epicsInt32 value) override;
    asynStatus readUInt32Digital(asynUser *u, epicsUInt32 *value, epicsUInt32 mask) override;
    asynStatus writeUInt32Digital(asynUser *u, epicsUInt32 value, epicsUInt32 mask) override;

private:
    bool ready(asynUser *u);
    void failed(asynUser *u, const std::exception &e);
    void ok();
    asynStatus acquire(asynUser *u);
    void opened();
    static double rangeOf(int index);
    static Term modeOf(int index);

    int pModel, pSerial, pStatus, pFirmware, pConnected;
    int pAiValue, pAiRange, pAiMode, pAiAverage, pAoValue, pDi, pDo, pDoRbv;
    int pWdOk;
    int pWdNumPoints, pWdFirstChan, pWdNumChans, pWdRange, pWdMode, pWdDwell, pWdDwellActual, pWdRun, pWdData, pWdTime;

    std::unique_ptr<Bus> bus_;
    std::unique_ptr<Device6008> dev_;
    epicsTimeStamp lastTry_;
    bool tried_ = false;
    int maxPoints_;
};

drvNiUsb6008::drvNiUsb6008(const char *port, const char *serial, int simulate, int maxPoints)
    : asynPortDriver(port, MAX_CHANNELS,
                     asynInt32Mask | asynFloat64Mask | asynUInt32DigitalMask | asynFloat64ArrayMask | asynOctetMask |
                         asynDrvUserMask,
                     asynInt32Mask | asynFloat64Mask | asynUInt32DigitalMask | asynFloat64ArrayMask | asynOctetMask,
                     ASYN_CANBLOCK | ASYN_MULTIDEVICE, 1, 0, 0),
      maxPoints_(maxPoints > 0 ? maxPoints : 16384)
{
    createParam("MODEL", asynParamOctet, &pModel);
    createParam("SERIAL", asynParamOctet, &pSerial);
    createParam("STATUS", asynParamOctet, &pStatus);
    createParam("FIRMWARE", asynParamOctet, &pFirmware);
    createParam("CONNECTED", asynParamInt32, &pConnected);
    createParam("AI_VALUE", asynParamFloat64, &pAiValue);
    createParam("AI_RANGE", asynParamInt32, &pAiRange);
    createParam("AI_MODE", asynParamInt32, &pAiMode);
    createParam("AI_AVERAGE", asynParamInt32, &pAiAverage);
    createParam("AO_VALUE", asynParamFloat64, &pAoValue);
    createParam("DI", asynParamUInt32Digital, &pDi);
    createParam("DO", asynParamUInt32Digital, &pDo);
    createParam("DO_RBV", asynParamUInt32Digital, &pDoRbv);
    createParam("WD_NUM_POINTS", asynParamInt32, &pWdNumPoints);
    createParam("WD_FIRST_CHAN", asynParamInt32, &pWdFirstChan);
    createParam("WD_NUM_CHANS", asynParamInt32, &pWdNumChans);
    createParam("WD_RANGE", asynParamInt32, &pWdRange);
    createParam("WD_MODE", asynParamInt32, &pWdMode);
    createParam("WD_DWELL", asynParamFloat64, &pWdDwell);
    createParam("WD_DWELL_ACTUAL", asynParamFloat64, &pWdDwellActual);
    createParam("WD_RUN", asynParamInt32, &pWdRun);
    createParam("WD_OK", asynParamInt32, &pWdOk);
    createParam("WD_DATA", asynParamFloat64Array, &pWdData);
    createParam("WD_TIME", asynParamFloat64Array, &pWdTime);

    for (int a = 0; a < MAX_CHANNELS; a++) {
        setIntegerParam(a, pAiRange, RANGE_10V);
        setIntegerParam(a, pAiMode, 0);
        setIntegerParam(a, pAiAverage, 1);
        setDoubleParam(a, pAiValue, 0.0);
        setDoubleParam(a, pAoValue, 0.0);
    }
    setIntegerParam(pWdNumPoints, 1000);
    setIntegerParam(pWdFirstChan, 0);
    setIntegerParam(pWdNumChans, 1);
    setIntegerParam(pWdRange, RANGE_10V);
    setIntegerParam(pWdMode, 0);
    setDoubleParam(pWdDwell, 0.001);
    setDoubleParam(pWdDwellActual, 0.001);
    setIntegerParam(pWdRun, 0);
    setIntegerParam(pWdOk, 1);
    setIntegerParam(pConnected, 0);
    setStringParam(pModel, "");
    setStringParam(pSerial, serial ? serial : "");
    setStringParam(pStatus, "");

    if (simulate) {
        auto fake = std::make_unique<Fake6008Bus>();
        fake->add(serial && *serial ? serial : "SIM");
        bus_ = std::move(fake);
        setStringParam(pFirmware, "simulated; the 6008 needs no firmware");
    } else {
        bus_ = std::make_unique<LibusbBus>([](uint16_t pid) { return pid == PID; });
        setStringParam(pFirmware, "none needed (USB-6008)");
    }
    dev_ = std::make_unique<Device6008>(*bus_, serial ? serial : "");
    ready(pasynUserSelf);
    callParamCallbacks();
}

bool drvNiUsb6008::ready(asynUser *u)
{
    if (dev_->isOpen())
        return true;
    epicsTimeStamp now;
    epicsTimeGetCurrent(&now);
    if (tried_ && epicsTimeDiffInSeconds(&now, &lastTry_) < RETRY_S) {
        epicsSnprintf(u->errorMessage, u->errorMessageSize, "%s: unit closed; next open attempt within %.0f s",
                      portName, RETRY_S);
        return false;
    }
    tried_ = true;
    lastTry_ = now;
    try {
        dev_->open();
        opened();
        setStringParam(pModel, MODEL);
        setStringParam(pSerial, dev_->serial());
        setIntegerParam(pConnected, 1);
        ok();
        callParamCallbacks();
        return true;
    } catch (const std::exception &e) {
        dev_->close();  // an open that failed part way leaves nothing open
        failed(u, e);
        return false;
    }
}

/* After each open: take P0 and P1 from the unit (reads only), so the records
 * show the lines as they are. */
void drvNiUsb6008::opened()
{
    uint8_t p1 = dev_->diRead(1);
    setUIntDigitalParam(pDo, p1, 0xF);
    setUIntDigitalParam(pDoRbv, p1, 0xF);
    setUIntDigitalParam(pDi, dev_->diRead(0), 0xFF);
}

void drvNiUsb6008::failed(asynUser *u, const std::exception &e)
{
    if (dynamic_cast<const TransportError *>(&e) || !dev_->isOpen()) {
        dev_->close();
        setIntegerParam(pConnected, 0);
    }
    setStringParam(pStatus, e.what());
    asynPrint(u, ASYN_TRACE_ERROR, "%s: %s\n", portName, e.what());
    callParamCallbacks();
    epicsSnprintf(u->errorMessage, u->errorMessageSize, "%s", e.what());
}

void drvNiUsb6008::ok()
{
    setStringParam(pStatus, "");
}

double drvNiUsb6008::rangeOf(int index)
{
    const std::vector<double> &r = diffRanges();
    if (index < 0 || size_t(index) >= r.size())
        throw Error("range index " + std::to_string(index));
    return r[index];
}

Term drvNiUsb6008::modeOf(int index)
{
    if (index == 0)
        return Term::RSE;
    if (index == 2)
        return Term::DIFF;
    throw Error("the 6008's modes are RSE (0) and DIFF (2), not " + std::to_string(index));
}

asynStatus drvNiUsb6008::readFloat64(asynUser *u, epicsFloat64 *value)
{
    int addr, reason = u->reason;
    getAddress(u, &addr);
    if (reason != pAiValue)
        return asynPortDriver::readFloat64(u, value);
    if (!ready(u))
        return asynError;
    try {
        int range, mode, avg;
        getIntegerParam(addr, pAiRange, &range);
        getIntegerParam(addr, pAiMode, &mode);
        getIntegerParam(addr, pAiAverage, &avg);
        *value = dev_->aiRead(addr, rangeOf(range), modeOf(mode), uint32_t(std::max(1, avg)));
        setDoubleParam(addr, pAiValue, *value);
        ok();
        callParamCallbacks(addr);
        return asynSuccess;
    } catch (const std::exception &e) {
        failed(u, e);
        return asynError;
    }
}

asynStatus drvNiUsb6008::writeFloat64(asynUser *u, epicsFloat64 value)
{
    int addr, reason = u->reason;
    getAddress(u, &addr);
    if (reason == pWdDwell) {
        if (!(value > 0)) {
            epicsSnprintf(u->errorMessage, u->errorMessageSize, "dwell must be > 0");
            return asynError;
        }
        double n = std::floor(CLOCK * value + 0.5);
        setDoubleParam(pWdDwell, value);
        setDoubleParam(pWdDwellActual, std::max(1.0, n) / CLOCK);
        callParamCallbacks();
        return asynSuccess;
    }
    if (reason != pAoValue)
        return asynPortDriver::writeFloat64(u, value);
    if (!ready(u))
        return asynError;
    try {
        dev_->aoWrite(addr, value);
        setDoubleParam(addr, pAoValue, std::min(AO_MAX, std::max(0.0, double(value))));
        ok();
        callParamCallbacks(addr);
        return asynSuccess;
    } catch (const std::exception &e) {
        failed(u, e);
        return asynError;
    }
}

asynStatus drvNiUsb6008::writeInt32(asynUser *u, epicsInt32 value)
{
    int addr, reason = u->reason;
    getAddress(u, &addr);
    if (reason == pAiRange || reason == pWdRange) {
        if (value < 0 || size_t(value) >= diffRanges().size())
            return asynError;
    } else if (reason == pAiMode || reason == pWdMode) {
        if (value != 0 && value != 2) {
            epicsSnprintf(u->errorMessage, u->errorMessageSize, "the 6008's modes are RSE (0) and DIFF (2)");
            return asynError;
        }
    } else if (reason == pWdRun) {
        if (!value)
            return asynSuccess;
        setIntegerParam(pWdRun, 1);
        callParamCallbacks();
        asynStatus st = acquire(u);
        setIntegerParam(pWdRun, 0);
        // Run returns to Done either way (a client waits on it); the outcome
        // stays in WaveDigOK, which alarms after a scan that did not run
        setIntegerParam(pWdOk, st == asynSuccess);
        callParamCallbacks();
        return st;
    }
    setIntegerParam(addr, reason, value);
    callParamCallbacks(addr);
    return asynSuccess;
}

asynStatus drvNiUsb6008::acquire(asynUser *u)
{
    if (!ready(u))
        return asynError;
    try {
        int n, first, nch, range, mode;
        double dwell;
        getIntegerParam(pWdNumPoints, &n);
        getIntegerParam(pWdFirstChan, &first);
        getIntegerParam(pWdNumChans, &nch);
        getIntegerParam(pWdRange, &range);
        getIntegerParam(pWdMode, &mode);
        getDoubleParam(pWdDwell, &dwell);
        if (n < 1 || n > maxPoints_)
            throw Error("WaveDig points must be 1.." + std::to_string(maxPoints_));
        if (nch < 1)
            throw Error("WaveDig needs a channel");
        if (n * dwell > MAX_SCAN_S)
            throw Error("WaveDig: " + std::to_string(n * dwell) + " s exceeds the " + std::to_string(int(MAX_SCAN_S)) +
                        " s a scan may hold the port");
        std::vector<int> chans;
        for (int c = first; c < first + nch; c++)
            chans.push_back(c);
        double r = rangeOf(range);
        Term t = modeOf(mode);
        std::vector<uint16_t> w = dev_->aiTimedWords(chans, 1.0 / dwell, uint32_t(n), r, t);
        double actual = Device6008::timedDivisor(chans.size(), 1.0 / dwell) / CLOCK;
        std::vector<double> tv(n), v(n);
        for (int i = 0; i < n; i++)
            tv[i] = i * actual;
        doCallbacksFloat64Array(tv.data(), n, pWdTime, 0);
        for (int k = 0; k < nch; k++) {
            for (int i = 0; i < n; i++)
                v[i] = dev_->scale(w[size_t(i) * nch + k], r, t, chans[k]);
            doCallbacksFloat64Array(v.data(), n, pWdData, chans[k]);
        }
        setDoubleParam(pWdDwellActual, actual);
        ok();
        return asynSuccess;
    } catch (const std::exception &e) {
        failed(u, e);
        return asynError;
    }
}

asynStatus drvNiUsb6008::readUInt32Digital(asynUser *u, epicsUInt32 *value, epicsUInt32 mask)
{
    int reason = u->reason;
    if (reason != pDi && reason != pDoRbv)
        return asynPortDriver::readUInt32Digital(u, value, mask);
    if (!ready(u))
        return asynError;
    try {
        if (reason == pDi) {
            uint8_t p0 = dev_->diRead(0);
            setUIntDigitalParam(pDi, p0, 0xFF);
            *value = p0 & mask;
        } else {
            uint8_t p1 = dev_->diRead(1);
            setUIntDigitalParam(pDoRbv, p1, 0xF);
            *value = p1 & mask;
        }
        ok();
        callParamCallbacks();
        return asynSuccess;
    } catch (const std::exception &e) {
        failed(u, e);
        return asynError;
    }
}

asynStatus drvNiUsb6008::writeUInt32Digital(asynUser *u, epicsUInt32 value, epicsUInt32 mask)
{
    if (u->reason != pDo)
        return asynPortDriver::writeUInt32Digital(u, value, mask);
    if (!ready(u))
        return asynError;
    try {
        uint8_t now = dev_->doWrite(uint8_t(value & mask & 0xF), uint8_t(mask & 0xF));
        setUIntDigitalParam(pDo, now, 0xF);
        setUIntDigitalParam(pDoRbv, now, 0xF);
        ok();
        callParamCallbacks();
        return asynSuccess;
    } catch (const std::exception &e) {
        failed(u, e);
        return asynError;
    }
}

/* niUsb6008Config(port, serial, simulate, maxPoints)
 * serial: the unit's USB serial, "" for the first 6008; simulate: 1 for the fake. */
extern "C" int niUsb6008Config(const char *port, const char *serial, int simulate, int maxPoints)
{
    try {
        new drvNiUsb6008(port, serial, simulate, maxPoints);
    } catch (const std::exception &e) {
        errlogPrintf("niUsb6008Config %s: %s\n", port, e.what());
        return -1;
    }
    return 0;
}

static const iocshArg arg0 = {"port", iocshArgString};
static const iocshArg arg1 = {"serial", iocshArgString};
static const iocshArg arg2 = {"simulate", iocshArgInt};
static const iocshArg arg3 = {"maxPoints", iocshArgInt};
static const iocshArg *const args[] = {&arg0, &arg1, &arg2, &arg3};
static const iocshFuncDef configDef = {"niUsb6008Config", 4, args};

static void configCall(const iocshArgBuf *a)
{
    niUsb6008Config(a[0].sval, a[1].sval, a[2].ival, a[3].ival);
}

static void niUsb6008Register()
{
    iocshRegister(&configDef, configCall);
}

extern "C" {
epicsExportRegistrar(niUsb6008Register);
}
