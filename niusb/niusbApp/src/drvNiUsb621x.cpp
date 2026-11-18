/* drvNiUsb621x: one NI USB-621x as an asyn port (ASYN_CANBLOCK: the port's
 * thread makes every USB call, one at a time).
 *
 * Parameters (addr = channel where it says so):
 *   MODEL, SERIAL, STATUS, FIRMWARE   octet: the unit, the last error ("" if
 *                                     none), the firmware folder's state
 *   CONNECTED                         int32
 *   AI_VALUE   float64, addr          an on-demand read, the mean of AI_AVERAGE samples
 *   AI_RANGE   int32, addr            0..3: +-10, 5, 1, 0.2 V
 *   AI_MODE    int32, addr            0 RSE, 1 NRSE, 2 DIFF
 *   AI_AVERAGE int32, addr
 *   AO_VALUE   float64, addr          AO0, AO1 (the 6211)
 *   DI         uint32Digital          P0.0-3
 *   DO         uint32Digital          P1.0-3 (masked writes)
 *   DO_RBV     uint32Digital          P1.0-3 as the unit reads them back
 *   WD_NUM_POINTS, WD_FIRST_CHAN, WD_NUM_CHANS, WD_RANGE, WD_MODE  int32
 *   WD_DWELL, WD_DWELL_ACTUAL         float64, s (the clock is 20 MHz / n)
 *   WD_RUN     int32                  1: one finite hardware-timed scan, then 0;
 *                                     an error (and the record's alarm) if none ran.
 *                                     A scan holds the port: at most MAX_SCAN_S
 *   WD_DATA    float64Array, addr     volts per channel; WD_TIME float64Array, s
 *
 * A transport error (the unit unplugged, a USB error) closes the unit; the
 * next request reopens it, at most once per RETRY_S, loading the firmware
 * again if it came back blank. */
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <asynPortDriver.h>
#include <epicsStdio.h>
#include <errlog.h>
#include <epicsExport.h>
#include <epicsTime.h>
#include <iocsh.h>

#include "NiUsb621x.h"
#include "NiUsbFake.h"
#include "NiUsbLibusb.h"

using namespace niusb;

static const double RETRY_S = 5.0;
static const double MAX_SCAN_S = 30.0;   // a scan holds the port thread; the other records wait
static const int MAX_CHANNELS = 16;

class drvNiUsb621x : public asynPortDriver {
public:
    drvNiUsb621x(const char *port, const char *serial, const char *fwDir, const char *simulate, int maxPoints);

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
    static Mode modeOf(int index);

    int pModel, pSerial, pStatus, pFirmware, pConnected;
    int pAiValue, pAiRange, pAiMode, pAiAverage, pAoValue, pDi, pDo, pDoRbv;
    int pWdNumPoints, pWdFirstChan, pWdNumChans, pWdRange, pWdMode, pWdDwell, pWdDwellActual, pWdRun, pWdData, pWdTime;

    std::unique_ptr<Bus> bus_;
    std::unique_ptr<Firmware> fw_;
    std::unique_ptr<Device> dev_;
    epicsTimeStamp lastTry_;
    bool tried_ = false;
    uint8_t doValue_ = 0;
    int maxPoints_;
};

drvNiUsb621x::drvNiUsb621x(const char *port, const char *serial, const char *fwDir, const char *simulate, int maxPoints)
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
    createParam("WD_DATA", asynParamFloat64Array, &pWdData);
    createParam("WD_TIME", asynParamFloat64Array, &pWdTime);

    for (int a = 0; a < MAX_CHANNELS; a++) {
        setIntegerParam(a, pAiRange, 0);
        setIntegerParam(a, pAiMode, 0);
        setIntegerParam(a, pAiAverage, 1);
        setDoubleParam(a, pAiValue, 0.0);
        setDoubleParam(a, pAoValue, 0.0);
    }
    setIntegerParam(pWdNumPoints, 1000);
    setIntegerParam(pWdFirstChan, 0);
    setIntegerParam(pWdNumChans, 1);
    setIntegerParam(pWdRange, 0);
    setIntegerParam(pWdMode, 0);
    setDoubleParam(pWdDwell, 0.001);
    setDoubleParam(pWdDwellActual, 0.001);
    setIntegerParam(pWdRun, 0);
    setIntegerParam(pConnected, 0);
    setStringParam(pModel, "");
    setStringParam(pSerial, serial ? serial : "");
    setStringParam(pStatus, "");

    std::string sim = simulate ? simulate : "";
    if (!sim.empty()) {
        auto fake = std::make_unique<FakeBus>();
        fake->add(sim == "USB-6210" ? 0x726F : 0x7270, serial && *serial ? serial : "SIM", true);
        bus_ = std::move(fake);
        setStringParam(pFirmware, "simulated: no firmware needed");
    } else {
        bus_ = std::make_unique<LibusbBus>();
        try {
            fw_ = std::make_unique<Firmware>(Firmware::load(fwDir ? fwDir : "/opt/ni-firmware"));
            setStringParam(pFirmware, std::string("ok: NI-DAQmx 20 files in ") + (fwDir ? fwDir : ""));
        } catch (const std::exception &e) {
            // a unit already loaded still works; a blank one will say why it cannot
            setStringParam(pFirmware, e.what());
        }
    }
    dev_ = std::make_unique<Device>(*bus_, serial ? serial : "", fw_.get());
    if (sim.empty())
        dev_->loadLock = fwDir ? fwDir : "/opt/ni-firmware";  // shared by every IOC on the host
    ready(pasynUserSelf);
    callParamCallbacks();
}

bool drvNiUsb621x::ready(asynUser *u)
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
        setStringParam(pModel, dev_->model().name);
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

/* After each open: take P1's state from the unit, so a masked write keeps
 * the other lines as they are; after a load from blank the AOs are at 0 V. */
void drvNiUsb621x::opened()
{
    uint32_t w = dev_->pfiRead();
    doValue_ = uint8_t((w >> DO_SHIFT) & 0xF);
    setUIntDigitalParam(pDo, doValue_, 0xF);
    setUIntDigitalParam(pDoRbv, doValue_, 0xF);
    setUIntDigitalParam(pDi, w & 0xF, 0xF);
    if (dev_->freshlyLoaded())
        for (int a = 0; a < MAX_CHANNELS; a++) {
            setDoubleParam(a, pAoValue, 0.0);
            callParamCallbacks(a);
        }
}

void drvNiUsb621x::failed(asynUser *u, const std::exception &e)
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

void drvNiUsb621x::ok()
{
    setStringParam(pStatus, "");
}

double drvNiUsb621x::rangeOf(int index)
{
    const std::vector<double> &r = ranges();
    if (index < 0 || size_t(index) >= r.size())
        throw Error("range index " + std::to_string(index));
    return r[index];
}

Mode drvNiUsb621x::modeOf(int index)
{
    static const Mode m[] = {Mode::RSE, Mode::NRSE, Mode::DIFF};
    if (index < 0 || index > 2)
        throw Error("mode index " + std::to_string(index));
    return m[index];
}

asynStatus drvNiUsb621x::readFloat64(asynUser *u, epicsFloat64 *value)
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
        *value = dev_->aiRead(addr, rangeOf(range), modeOf(mode), std::max(1, avg));
        setDoubleParam(addr, pAiValue, *value);
        ok();
        callParamCallbacks(addr);
        return asynSuccess;
    } catch (const std::exception &e) {
        failed(u, e);
        return asynError;
    }
}

asynStatus drvNiUsb621x::writeFloat64(asynUser *u, epicsFloat64 value)
{
    int addr, reason = u->reason;
    getAddress(u, &addr);
    if (reason == pWdDwell) {
        if (!(value > 0)) {
            epicsSnprintf(u->errorMessage, u->errorMessageSize, "dwell must be > 0");
            return asynError;
        }
        double n = std::round(TIMEBASE * value);
        setDoubleParam(pWdDwell, value);
        setDoubleParam(pWdDwellActual, std::max(1.0, n) / TIMEBASE);
        callParamCallbacks();
        return asynSuccess;
    }
    if (reason != pAoValue)
        return asynPortDriver::writeFloat64(u, value);
    if (!ready(u))
        return asynError;
    try {
        dev_->aoWrite(addr, value);
        setDoubleParam(addr, pAoValue, value);
        ok();
        callParamCallbacks(addr);
        return asynSuccess;
    } catch (const std::exception &e) {
        failed(u, e);
        return asynError;
    }
}

asynStatus drvNiUsb621x::writeInt32(asynUser *u, epicsInt32 value)
{
    int addr, reason = u->reason;
    getAddress(u, &addr);
    if (reason == pAiRange || reason == pWdRange) {
        if (value < 0 || size_t(value) >= ranges().size())
            return asynError;
    } else if (reason == pAiMode || reason == pWdMode) {
        if (value < 0 || value > 2)
            return asynError;
    } else if (reason == pWdRun) {
        if (!value)
            return asynSuccess;
        setIntegerParam(pWdRun, 1);
        callParamCallbacks();
        asynStatus st = acquire(u);
        setIntegerParam(pWdRun, 0);
        callParamCallbacks();
        return st;
    }
    setIntegerParam(addr, reason, value);
    callParamCallbacks(addr);
    return asynSuccess;
}

asynStatus drvNiUsb621x::acquire(asynUser *u)
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
        std::vector<int16_t> codes = dev_->aiTimedCodes(chans, 1.0 / dwell, uint32_t(n), r, modeOf(mode));
        double actual = 1.0 / dev_->aiTimedRate(1.0 / dwell);
        std::vector<double> t(n), v(n);
        for (int i = 0; i < n; i++)
            t[i] = i * actual;
        doCallbacksFloat64Array(t.data(), n, pWdTime, 0);
        for (int k = 0; k < nch; k++) {
            for (int i = 0; i < n; i++)
                v[i] = dev_->scale(codes[size_t(i) * nch + k], r);
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

asynStatus drvNiUsb621x::readUInt32Digital(asynUser *u, epicsUInt32 *value, epicsUInt32 mask)
{
    int reason = u->reason;
    if (reason != pDi && reason != pDoRbv)
        return asynPortDriver::readUInt32Digital(u, value, mask);
    if (!ready(u))
        return asynError;
    try {
        uint32_t w = dev_->pfiRead();
        setUIntDigitalParam(pDi, w & 0xF, 0xF);
        setUIntDigitalParam(pDoRbv, (w >> DO_SHIFT) & 0xF, 0xF);
        *value = (reason == pDi ? w : w >> DO_SHIFT) & 0xF & mask;
        ok();
        callParamCallbacks();
        return asynSuccess;
    } catch (const std::exception &e) {
        failed(u, e);
        return asynError;
    }
}

asynStatus drvNiUsb621x::writeUInt32Digital(asynUser *u, epicsUInt32 value, epicsUInt32 mask)
{
    if (u->reason != pDo)
        return asynPortDriver::writeUInt32Digital(u, value, mask);
    if (!ready(u))
        return asynError;
    try {
        uint8_t next = uint8_t(((doValue_ & ~mask) | (value & mask)) & 0xF);
        dev_->doWrite(next);
        doValue_ = next;
        setUIntDigitalParam(pDo, next, 0xF);
        ok();
        callParamCallbacks();
        return asynSuccess;
    } catch (const std::exception &e) {
        failed(u, e);
        return asynError;
    }
}

/* niUsb621xConfig(port, serial, fwDir, simulate, maxPoints)
 * simulate: "" for hardware, or "USB-6211" / "USB-6210" for the fake. */
extern "C" int niUsb621xConfig(const char *port, const char *serial, const char *fwDir, const char *simulate,
                               int maxPoints)
{
    try {
        new drvNiUsb621x(port, serial, fwDir, simulate, maxPoints);
    } catch (const std::exception &e) {
        errlogPrintf("niUsb621xConfig %s: %s\n", port, e.what());
        return -1;
    }
    return 0;
}

static const iocshArg arg0 = {"port", iocshArgString};
static const iocshArg arg1 = {"serial", iocshArgString};
static const iocshArg arg2 = {"fwDir", iocshArgString};
static const iocshArg arg3 = {"simulate", iocshArgString};
static const iocshArg arg4 = {"maxPoints", iocshArgInt};
static const iocshArg *const args[] = {&arg0, &arg1, &arg2, &arg3, &arg4};
static const iocshFuncDef configDef = {"niUsb621xConfig", 5, args};

static void configCall(const iocshArgBuf *a)
{
    niUsb621xConfig(a[0].sval, a[1].sval, a[2].sval, a[3].sval, a[4].ival);
}

static void niUsb621xRegister()
{
    iocshRegister(&configDef, configCall);
}

extern "C" {
epicsExportRegistrar(niUsb621xRegister);
}
