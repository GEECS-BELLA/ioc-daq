"""NI USB-621x over pyusb: the reference for the C++ driver (niusb/).

Bring-up, calibration, AI (on demand and finite timed), AO, DI, DO, decoded
from USB captures of NI-DAQmx 20 and a USB-6211, and run from Linux on a
USB-6211 and a USB-6210. tests/test_cpp.py holds the C++ driver to the same
bytes. NI's firmware is read at run time from a folder of five NI-DAQmx files
(FIRMWARE_FILES; geecs-epics decision 23) and never stored with this code.

Message on bulk ep01:
    00 ch | len(2) | len-4(2) | type | verb | 03 | subsys | index(2) | payload
A type-1 message gets exactly one ep81 reply on the same channel; reply byte 11
is the status (02 ok, 06 error) and data words follow from byte 12. A type-0
message announces a bulk transfer and gets no reply.
"""
import hashlib
import os
import struct
import time

try:
    import usb.core
    import usb.util
except ImportError:  # the tests run the protocol without pyusb
    usb = None

VID, BLANK = 0x3923, 0x7269
# After the firmware load each model renumerates under its own product ID.
LOADED = {0x7270: "USB-6211", 0x726F: "USB-6210"}


# NI-DAQmx 20's files (NI-DAQ\\fpgaFiles\\miousb\\) and their SHA-256.
FIRMWARE_FILES = {
    "usb621x.cfg": "f890e2c578976adbc0e47a1636a0dc8e8d21b69e01a385da9f30199b7a35f8e1",
    "usb621x0.cfg": "da1a4fe26435cbc20a8814c5e57c8ae861f69eb45211277bfa4fa6b0fa9b047a",
    "usb621x1.cfg": "a1d002660ac1a1044b51eb95b68c34dafa8b58877906d9a2114f6d37c53e877d",
    "SlicFpga.cfg": "2fcb1fb78ae454cbb328199e6a9e0e07b8a96423511196f6fb9f6863a1ee0be6",
    "STC3Lite.cfg": "3b72c4056816f37de5aa6940b207a3b638bd65b0eec50a4543f0d7f9e3d77af1",
}


def load_firmware(fwdir, check=True):
    """NI's five files -> the six images the load sends; a differing file is refused by name."""
    raw = {}
    for name, want in FIRMWARE_FILES.items():
        with open(os.path.join(fwdir, name), "rb") as f:
            raw[name] = f.read()
        got = hashlib.sha256(raw[name]).hexdigest()
        if check and got != want:
            raise Error("%s in %s is not the NI-DAQmx 20 file (sha256 %s, expected %s)" % (name, fwdir, got, want))
    m0, m1 = raw["usb621x0.cfg"], raw["usb621x1.cfg"]
    if m0[:16384] != m1[:16384]:
        raise Error("usb621x0.cfg and usb621x1.cfg disagree on the main image")
    return {"loader": raw["usb621x.cfg"], "main16k": m0[:16384], "bank0": m0[16384:], "bank1": m1[16384:],
            "fpga0": raw["SlicFpga.cfg"], "fpga1": raw["STC3Lite.cfg"]}


def _loaded():
    return [d for d in usb.core.find(find_all=True, idVendor=VID) if d.idProduct in LOADED]

# M-series (STC3) AI config word: bit 14 last channel, bits 9-11 gain code,
# bits 6-8 terminal type, bits 0-3 channel.
GAIN = {10.0: 0, 5.0: 1, 1.0: 4, 0.2: 5}       # 621x range -> gain code
CAL_SLOT = {10.0: 0, 5.0: 1, 1.0: 3, 0.2: 5}   # pair in the EEPROM self-cal table
MODE = {"DIFF": 1, "NRSE": 2, "RSE": 3}
SELF_CAL_TABLE = 0x1E4                         # 7 x (gain V/code, offset V), BE float32
AO_CAL = {0: 0x22D, 1: 0x25E}                  # AO0, AO1 (6211): (codes/V, offset codes), BE float32
TIMEBASE = 20e6                                # sample clock = TIMEBASE / divisor
# The PFI word: P0.0-3 (inputs) are bits 0-3, P1.0-3 (outputs) bits 4-7.
DO_SHIFT = 4
AI_ONDEMAND_ROUTE = bytes.fromhex("00010000000112000101130000011c000001010000000001010000000101000001000000")
AI_TIMED_ROUTE = bytes.fromhex("00010000000100000101130000011c000000010000000001010100000101000000000100")

# Sent by NI-DAQmx after both FPGAs report done: the first two, its EEPROM
# read, then the rest (POST_FPGA_HEAD = POST_FPGA[:2]).
POST_FPGA = [
    "0000000c0008010000000000", "0000000c0008010904010000",
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
]


def msg(ch, verb, subsys, index=0, payload=b"", kind=1):
    body = bytes([kind, verb, 0x03, subsys]) + struct.pack(">H", index) + payload
    n = len(body) + 6
    return struct.pack(">HHH", ch, n, n - 4) + body


def announce(ch, ep, nbytes):
    """Type-0 message: the next bulk transfer of nbytes on pipe ep (no reply)."""
    body = bytes.fromhex("00080002") + struct.pack(">HI", ep, nbytes)
    n = len(body) + 6
    return struct.pack(">HHH", ch, n, n - 4) + body


def ai_config(chans, vrange, mode):
    """AI channel list: u16 byte count, one config word per channel (bit 14 on the last), padded to 4."""
    words = [(GAIN[vrange] << 9) | (MODE[mode] << 6) | (c & 0xF) for c in chans]
    words[-1] |= 0x4000
    body = struct.pack(">%dH" % len(words), *words)
    out = struct.pack(">H", 2 + len(body)) + body
    return out + b"\0" * (-len(out) % 4)


def ai_setup_msgs(chans, vrange, mode, divisor=0, n=0):
    """Messages up to the start: divisor=0 is on-demand, else finite timed n samples per channel."""
    if isinstance(chans, int):
        chans = [chans]
    timing = (struct.pack(">IIIIII", 3, 0x118, 2, divisor, n - 1, 0xFFFFFFFF) if divisor
              else struct.pack(">IIIIII", 3, 0x118, 0, 0, 0xFFFFFFFF, 0xFFFFFFFF))
    head = [
        msg(1, 0x0A, 0x02, payload=bytes.fromhex("00320000")),
        msg(1, 0x08, 0x15, payload=bytes.fromhex("0000000008000000")),
        msg(1, 0x0B, 0x12),
        msg(1, 0x0A, 0x12, payload=ai_config(chans, vrange, mode)),
        msg(1, 0x0C, 0x12),
        msg(1, 0x10, 0x13, payload=timing),
        msg(1, 0x11, 0x13, payload=AI_TIMED_ROUTE if divisor else AI_ONDEMAND_ROUTE),
        msg(1, 0x08, 0x10),
        msg(1, 0x09, 0x11, payload=bytes.fromhex("00000002")),
    ]
    if divisor:  # announce the whole acquisition on pipe 8, then arm
        head.append(announce(7, 0x0008, 2 * n * len(chans)))
    head += [msg(1, 0x0D, 0x11), msg(1, 0x0A, 0x13)]
    if divisor:
        head.append(msg(0, 0x04, 0x15, payload=bytes.fromhex("00040001")))
    head += [
        msg(1, 0x0C, 0x15),
        msg(1, 0x0A, 0x11),
        msg(1, 0x09, 0x10),
        msg(1, 0x0B, 0x13, payload=bytes.fromhex("00000000")),
    ]
    return head


AI_SAMPLE = [announce(1, 0x0008, 2), msg(1, 0x0C, 0x13)]
AI_TEARDOWN = [msg(1, v, s) for v, s in ((0x0D, 0x13), (0x0E, 0x13), (0x0A, 0x10), (0x0B, 0x11),
                                          (0x0D, 0x15), (0x0C, 0x11), (0x0B, 0x10), (0x0F, 0x13),
                                          (0x09, 0x15))]
AI_TIMED_TEARDOWN = AI_TEARDOWN[:5] + [msg(0, 0x05, 0x15, payload=bytes.fromhex("00040001"))] + AI_TEARDOWN[5:]


class Error(Exception):
    pass


def usb_errors():
    """The transport's errors: after one, no teardown is sent (the C++ driver's rule)."""
    return (usb.core.USBError,) if usb is not None else ()


BRINGUP_HEAD = ["0000000c0008010000000000", "0000000c0008010200000000", "0000000c0008010000000000"]


class USB621x:
    def __init__(self, fwdir=None, timeout=10.0, serial=None, port=None):
        """Open one unit: a blank one is loaded first (if fwdir is given).

        With several units, pick by serial (loaded units only, e.g. "016CD81D") or by
        port, the (bus, port path) tuple, which survives the renumeration.
        """
        self.fwdir, self.timeout = fwdir, timeout
        self.fw = load_firmware(fwdir) if fwdir else None
        here = lambda d: port is None or (d.bus, tuple(d.port_numbers or ())) == port
        blank = None if serial else next((d for d in usb.core.find(find_all=True, idVendor=VID, idProduct=BLANK) if here(d)), None)
        if blank is not None:
            if fwdir is None:
                raise Error("blank device (3923:7269) and no firmware directory given")
            where = (blank.bus, tuple(blank.port_numbers or ()))
            self._fx2(blank)
            self.d = self._wait_loaded(where)
            self.model = LOADED[self.d.idProduct]
            self._claim()
            self._bringup()  # reads the EEPROM in NI-DAQmx's place
        else:
            for d in _loaded():
                if here(d) and (serial is None or usb.util.get_string(d, d.iSerialNumber) == serial):
                    self.d = d
                    self.model = LOADED[d.idProduct]
                    break
            else:
                raise Error("no NI USB-621x found")
            self._claim()
            self._read_eeprom()
        self.where = (self.d.bus, tuple(self.d.port_numbers or ()))
        self.serial = usb.util.get_string(self.d, self.d.iSerialNumber)
        self.do_value = 0

    @classmethod
    def adopt(cls, dev, eeprom, model="USB-6211", timeout=10.0):
        """For the tests: an object on an already open device and a known EEPROM, no I/O."""
        self = cls.__new__(cls)
        self.d, self.timeout, self.fw, self.model, self.do_value = dev, timeout, None, model, 0
        self._set_eeprom(eeprom)
        return self

    def _set_eeprom(self, eeprom):
        self.eeprom = eeprom
        t = eeprom[SELF_CAL_TABLE:SELF_CAL_TABLE + 56]
        self.cal = [struct.unpack(">ff", t[8 * i:8 * i + 8]) for i in range(7)]
        ao = AO_CAL if getattr(self, "model", "USB-6211") == "USB-6211" else {}
        self.ao_cal = {ch: struct.unpack(">ff", eeprom[a:a + 8]) for ch, a in ao.items()}

    # ---------------------------------------------------------------- transport
    def _img(self, name):
        if self.fw is None:
            raise Error("no NI firmware folder given")
        return self.fw[name]

    def _claim(self):
        if self.d.is_kernel_driver_active(0):
            self.d.detach_kernel_driver(0)
        try:
            cfg = self.d.get_active_configuration()
        except usb.core.USBError:
            cfg = None
        if cfg is None:  # a second SET_CONFIGURATION wedges this firmware
            self.d.set_configuration()
        usb.util.claim_interface(self.d, 0)

    def send(self, m):
        """Send one message; return its reply (type 1) or None (type 0)."""
        self.d.write(0x01, m, timeout=15000)  # 08 03 ff blocks ~3.3 s
        if m[6] != 1:
            return None
        while True:
            r = bytes(self.d.read(0x81, 512, timeout=int(self.timeout * 1000)))
            if r[:2] == m[:2]:
                return r

    def check(self, m):
        r = self.send(m)
        if r is not None and r[11] != 2:
            raise Error("%s -> status %02x (%s)" % (m.hex(" "), r[11], r.hex(" ")))
        return r

    # ---------------------------------------------------------------- bring-up
    def _fx2_write(self, dev, req, base, data):
        for off in range(0, len(data), 512):
            c = data[off:off + 512]
            dev.ctrl_transfer(0x40, req, base + off, 0, c, timeout=2000)
            if bytes(dev.ctrl_transfer(0xC0, req, base + off, 0, len(c), timeout=2000)) != c:
                raise Error("FX2 readback mismatch, request %#x at %#x" % (req, base + off))

    def _fx2(self, dev):
        def cpucs(hold):
            dev.ctrl_transfer(0x40, 0xA0, 0xE600, 0, bytes([hold]), timeout=2000)
        cpucs(1)
        self._fx2_write(dev, 0xA0, 0, self._img("loader"))
        cpucs(0)
        time.sleep(0.05)
        dev.ctrl_transfer(0xC0, 0xB0, 0, 0, 2, timeout=2000)  # the loader's version, as NI-DAQmx asks
        self._fx2_write(dev, 0xA4, 0x4000, self._img("bank0"))
        self._fx2_write(dev, 0xA5, 0x4000, self._img("bank1"))
        cpucs(1)
        self._fx2_write(dev, 0xA0, 0, self._img("main16k"))
        cpucs(0)
        usb.util.dispose_resources(dev)

    def _wait_loaded(self, where):
        for _ in range(100):
            time.sleep(0.2)
            d = next((d for d in _loaded()
                      if (d.bus, tuple(d.port_numbers or ())) == where), None)
            if d is not None:
                time.sleep(1.0)
                return d
        raise Error("device did not renumerate with a known 621x product ID")

    def _fpga(self, target, image):
        data = self._img(image)
        if target == 0xFF:  # length announce, as NI-DAQmx sends before the Xilinx
            self.send(announce(0, 0x0002, len(data)))
        self.check(msg(0, 0x08, target))
        for off in range(0, len(data), 512):
            self.d.write(0x06, data[off:off + 512], timeout=5000)
        for _ in range(100):
            r = self.check(msg(0, 0x09, target))
            if len(r) == 12 or r[12] == 1:  # FPGA0 answers a data word, FPGA1 none
                return
            time.sleep(0.05)
        raise Error("FPGA %#x never reported done" % target)

    def _bringup(self):
        for m in BRINGUP_HEAD:
            self.send(bytes.fromhex(m))
        self._fpga(0xFE, "fpga0")
        self._fpga(0xFF, "fpga1")
        for m in POST_FPGA[:2]:
            self.send(bytes.fromhex(m))
        self._read_eeprom()
        for m in POST_FPGA[2:]:
            self.check(bytes.fromhex(m))

    def _read_eeprom(self):
        self.check(msg(1, 0x0C, 0x05))
        out = b""
        for off in range(0, 1024, 0x2B):
            n = min(0x2B, 1024 - off)
            r = self.check(msg(1, 0x08, 0x05, payload=struct.pack(">II", off, n)))
            out += r[14:14 + n]
        self._set_eeprom(out)
        return out

    # ---------------------------------------------------------------- AI
    def ai_read(self, chan=0, vrange=10.0, mode="RSE", n=1):
        """Software-timed reads of one channel: (volts, raw codes), self-cal scaled."""
        if vrange not in GAIN:
            raise Error("6211 ranges are %s" % sorted(GAIN))
        for m in ai_setup_msgs(chan, vrange, mode):
            self.check(m)
        codes = []
        try:
            for _ in range(n):
                for m in AI_SAMPLE:
                    self.check(m)
                codes.append(struct.unpack("<h", bytes(self.d.read(0x88, 512, timeout=3000)))[0])
        except usb_errors():
            raise  # the unit is gone or wedged: no teardown to send
        except BaseException:
            for m in AI_TEARDOWN:
                self.send(m)
            raise
        for m in AI_TEARDOWN:
            self.send(m)
        g, o = self.cal[CAL_SLOT[vrange]]
        return [g * k + o for k in codes], codes

    def ai_timed(self, chans, rate, n, vrange=10.0, mode="RSE"):
        """Finite hardware-timed scan: n samples per channel at rate (Hz) on the 20 MHz timebase.

        Returns {chan: [volts]} (self-cal scaled) and the interleaved raw codes.
        """
        if isinstance(chans, int):
            chans = [chans]
        divisor = int(round(TIMEBASE / rate))
        if not 1 <= n <= 0xFFFFFFFF or divisor < 1:
            raise Error("bad timing: rate %g n %d" % (rate, n))
        for m in ai_setup_msgs(chans, vrange, mode, divisor, n):
            self.check(m)
        want = 2 * n * len(chans)
        raw = b""
        try:
            self.check(msg(1, 0x0C, 0x13))  # start
            deadline = time.time() + n / rate + 5.0
            while len(raw) < want:
                if time.time() > deadline:
                    raise Error("timed AI: %d of %d bytes" % (len(raw), want))
                raw += bytes(self.d.read(0x88, min(want - len(raw), 1 << 16), timeout=int((n / rate + 5) * 1000)))
        except usb_errors():
            raise
        except BaseException:
            for m in AI_TIMED_TEARDOWN:
                self.send(m)
            raise
        for m in AI_TIMED_TEARDOWN:
            self.send(m)
        codes = list(struct.unpack("<%dh" % (want // 2), raw[:want]))
        g, o = self.cal[CAL_SLOT[vrange]]
        k = len(chans)
        return {c: [g * x + o for x in codes[i::k]] for i, c in enumerate(chans)}, codes

    # ---------------------------------------------------------------- AO
    def ao_write(self, chan, volts):
        """Static AO update, scaled with the device's AO calibration from the EEPROM."""
        if chan not in self.ao_cal:
            raise Error("AO channel %d has no calibration entry yet" % chan)
        if not -10.0 <= volts <= 10.0:
            raise Error("AO range is +-10 V")
        g, o = self.ao_cal[chan]
        code = max(-32768, min(32767, int(round(g * volts + o))))
        self.check(msg(1, 0x08, 0x20, chan, bytes.fromhex("0001000000000000")))
        self.check(msg(1, 0x0A, 0x20, chan, struct.pack(">Hh", 0x0004, code)))
        self.check(msg(1, 0x09, 0x20, chan))
        return code

    # ---------------------------------------------------------------- DIO
    def pfi_read(self):
        """The whole PFI word: P0.0-3 in bits 0-3, P1.0-3 (output readback) in bits 4-7."""
        r = self.check(msg(1, 0x08, 0x28, index=2))
        return struct.unpack(">I", r[12:16])[0]

    def di_read(self):
        """Port 0 (P0.0-P0.3 on the 6211) as an integer."""
        return self.pfi_read() & 0xF

    def do_write(self, value):
        """Port 1 (P1.0-P1.3 on the 6211): set all four static outputs to value."""
        if not 0 <= value <= 0xF:
            raise Error("port 1 is 4 lines")
        self.check(msg(1, 0x0A, 0x28, 3, bytes.fromhex("000000f001000000")))
        self.check(msg(1, 0x09, 0x28, 3, bytes.fromhex("000000f0") + struct.pack(">I", value << DO_SHIFT)))
        self.do_value = value
