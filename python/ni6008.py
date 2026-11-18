"""NI USB-6008 over pyusb: the reference for the C++ driver (niusb/, NiUsb6008).

On-demand and finite timed AI, static AO, DI on port 0 and DO on port 1,
decoded from USB captures of NI-DAQmx 20 and a USB-6008. The unit needs no
firmware: it enumerates ready (3923:717A). tests/test_cpp.py holds the C++
driver to the same bytes as this module and as NI-DAQmx
(tests/fixtures/daqmx-6008.json).

Message on bulk ep01 (the 621x's framing with family byte 02):
    00 ch | len(2) | len-4(2) | kind | verb | 02 | subsys | index(2) | payload
A kind-1 message gets one ep81 reply on the same channel; reply byte 11 is the
status (02 ok) and data follow from byte 12. A kind-0 message asks for AI
samples, which come on bulk ep82 as big-endian 16-bit words (the 12-bit code
in the top 12 bits); it gets no reply.

Subsystems: 00 AI timing and acquisition, 02 AI channels, 03 AI data,
04 calibration memory, 10 DIO, 21 AO.

Opening the unit sends nothing but the calibration read: no AO write, no DIO
direction. An IOC restart leaves the outputs as they are (NI-DAQmx zeroes
both AOs and resets the DIO directions when the unit is plugged in; that is
not repeated here).
"""
import math
import struct
import time

try:
    import usb.core
    import usb.util
except ImportError:  # the tests run the protocol without pyusb
    usb = None

VID, PID = 0x3923, 0x717A
MODEL = "USB-6008"
EP_CMD, EP_REPLY, EP_DATA = 0x01, 0x81, 0x82
FAMILY = 0x02
SUB_AI_TIMING, SUB_AI_CHAN, SUB_AI_DATA, SUB_CAL, SUB_DIO, SUB_AO = 0x00, 0x02, 0x03, 0x04, 0x10, 0x21

# The calibration memory: 512 bytes, read 32 at a time. Big-endian float32
# pairs (volts at the low end, volts at the high end of a range).
CAL_SIZE, CAL_CHUNK = 512, 0x20
DIFF_RANGES = (20.0, 10.0, 5.0, 4.0, 2.5, 2.0, 1.25, 1.0)   # +-V, in the memory's order
CAL_DIFF, CAL_RSE, CAL_AO = 0x10, 0x110, 0x150             # DIFF: + 0x20 * range + 8 * chan
RSE_RANGE = 10.0                                            # the only RSE range
N_RSE, N_DIFF, N_AO = 8, 4, 2
AO_MAX = 5.0                                                # AO 0..5 V
MODE_CODE = {"RSE": 0x02, "DIFF": 0x00}
CLOCK = 24e6                       # the AI scan clock is CLOCK / divisor
MAX_DIVISOR = 0xFFFF               # the larger divisors DAQmx would send are not captured
MAX_AI_RATE = 10e3                 # aggregate samples/s
POLL_S = 0.05                      # NI-DAQmx's status poll during a timed read


class Error(Exception):
    pass


def usb_errors():
    """The transport's errors: after one, no teardown is sent (the C++ driver's rule)."""
    return (usb.core.USBError,) if usb is not None else ()


def timeouts():
    """A read that timed out: pyusb's, or the replay's TimeoutError."""
    t = (TimeoutError,)
    if usb is not None:
        t += (usb.core.USBTimeoutError,)
    return t


def msg(verb, subsys, payload=b"", index=0, kind=1, ch=1):
    body = bytes([kind, verb, FAMILY, subsys]) + struct.pack(">H", index) + payload
    n = len(body) + 6
    return struct.pack(">HHH", ch, n, n - 4) + body


def pad4(b):
    return b + b"\0" * (-len(b) % 4)


def cal_read(addr, n=CAL_CHUNK):
    return msg(0x14, SUB_CAL, struct.pack(">IHH", addr, n, 0))


def chan_list(chans):
    """u16 byte count (2 + 2 per channel), then the channels as u16 (unpadded)."""
    chans = list(chans)
    return struct.pack(">H%dH" % len(chans), 2 + 2 * len(chans), *chans)


def ai_config_msgs(chans, vrange, mode):
    """The channel list, the terminal modes and the range (one range for every channel).

    The range message names the entries it applies to by their position in
    the channel list (0, 1, ...), not by channel number (ai1 alone: 0).
    """
    mv = int(round(vrange * 1000))
    modes = struct.pack(">H", 2 + len(chans)) + bytes([MODE_CODE[mode]] * len(chans))
    return [
        msg(0x0F, SUB_AI_CHAN, pad4(chan_list(chans))),
        msg(0x0E, SUB_AI_CHAN, pad4(modes)),
        msg(0x10, SUB_AI_CHAN, pad4(struct.pack(">ii", mv, -mv) + b"\xfd\xfd" + chan_list(range(len(chans))))),
    ]


def ai_ondemand_msgs(chans, vrange, mode, n):
    """Everything up to and including the request for n samples (kind 0)."""
    return ai_config_msgs(chans, vrange, mode) + [
        msg(0x0E, SUB_AI_TIMING, struct.pack(">I", 0)),
        msg(0x0F, SUB_AI_TIMING, struct.pack(">I", 1)),
        msg(0x13, SUB_AI_TIMING), msg(0x15, SUB_AI_TIMING), msg(0x18, SUB_AI_TIMING),
        msg(0x0F, SUB_AI_DATA), msg(0x09, SUB_AI_DATA), msg(0x09, SUB_AI_TIMING),
        msg(0x14, SUB_AI_DATA, struct.pack(">I", n), kind=0),
    ]


def ai_timed_msgs(chans, vrange, mode, divisor, n):
    """Everything up to and including the start (0a 02 00) of n scans on CLOCK / divisor."""
    return ai_config_msgs(chans, vrange, mode) + [
        msg(0x0E, SUB_AI_TIMING, struct.pack(">I", 0x01000000)),
        msg(0x16, SUB_AI_TIMING, struct.pack(">I", divisor)),
        msg(0x0F, SUB_AI_TIMING, struct.pack(">I", n)),
        msg(0x13, SUB_AI_TIMING), msg(0x15, SUB_AI_TIMING), msg(0x18, SUB_AI_TIMING),
        msg(0x12, SUB_AI_DATA, struct.pack(">I", 2 * n * len(chans)), kind=0),
        msg(0x0F, SUB_AI_DATA), msg(0x09, SUB_AI_DATA), msg(0x09, SUB_AI_TIMING),
        msg(0x0A, SUB_AI_TIMING),
    ]


# NI-DAQmx's poll while a timed read runs, every 50 ms until the samples are in.
AI_POLL = [msg(0x13, SUB_AI_DATA), msg(0x10, SUB_AI_DATA)]
AI_TEARDOWN = [msg(0x0B, SUB_AI_TIMING), msg(0x0C, SUB_AI_TIMING), msg(0x0C, SUB_AI_DATA),
               msg(0x0D, SUB_AI_TIMING)]


def dio_read_msg(port):
    return msg(0x0E, SUB_DIO, bytes([0, 3, port, 0]))


def dio_write_msg(port, value):
    return msg(0x0F, SUB_DIO, bytes([0, 3, port, 0, 3, value & 0xFF, 0, 0]))


def dio_direction_msg(p0, p1):
    """Line directions, 1 = output: P0's 8 lines, P1's 4."""
    return msg(0x12, SUB_DIO, bytes([0, 4, p0 & 0xFF, p1 & 0x0F, 0, 4, 0, 0]))


def ao_msg(chan, code):
    return msg(0x0E, SUB_AO, struct.pack(">hH", code, 0), index=chan)


def parse_cal(mem):
    """{"diff": {(range, chan): (lo, hi)}, "rse": {chan: (lo, hi)}, "ao": {chan: (lo, hi)}}."""
    if len(mem) < CAL_SIZE:
        raise Error("calibration memory of %d bytes" % len(mem))
    pair = lambda a: struct.unpack(">ff", mem[a:a + 8])
    return {
        "diff": {(r, c): pair(CAL_DIFF + 0x20 * i + 8 * c) for i, r in enumerate(DIFF_RANGES) for c in range(N_DIFF)},
        "rse": {c: pair(CAL_RSE + 8 * c) for c in range(N_RSE)},
        "ao": {c: pair(CAL_AO + 8 * c) for c in range(N_AO)},
    }


def round_half_up(x):
    return int(math.floor(x + 0.5))


def check_channels(chans, vrange, mode):
    if mode not in MODE_CODE:
        raise Error("the 6008's modes are RSE and DIFF, not %s" % mode)
    if mode == "RSE" and vrange != RSE_RANGE:
        raise Error("the 6008's RSE range is +-10 V only")
    if mode == "DIFF" and vrange not in DIFF_RANGES:
        raise Error("the 6008's DIFF ranges are %s" % (DIFF_RANGES,))
    n = N_RSE if mode == "RSE" else N_DIFF
    for c in chans:
        if not 0 <= c < n:
            raise Error("AI channel %d out of range for the 6008 in %s" % (c, mode))


class USB6008:
    def __init__(self, serial=None, timeout=10.0):
        """Open one unit (by its USB serial, e.g. "019CDC74", or the first) and read its calibration."""
        self.timeout = timeout
        for d in usb.core.find(find_all=True, idVendor=VID, idProduct=PID):
            if serial is None or usb.util.get_string(d, d.iSerialNumber) == serial:
                self.d = d
                break
        else:
            raise Error("no NI USB-6008%s found" % ("" if serial is None else " with serial " + serial))
        self.serial = usb.util.get_string(self.d, self.d.iSerialNumber)
        self._claim()
        self._attach()

    @classmethod
    def adopt(cls, dev, mem, timeout=10.0):
        """For the tests: an object on an already open device and a known calibration memory, no I/O."""
        self = cls.__new__(cls)
        self.d, self.timeout, self.serial = dev, timeout, None
        self._dir_set = False
        self._set_cal(mem)
        return self

    # ---------------------------------------------------------------- transport
    def _claim(self):
        if self.d.is_kernel_driver_active(0):
            self.d.detach_kernel_driver(0)
        try:
            cfg = self.d.get_active_configuration()
        except usb.core.USBError:
            cfg = None
        if cfg is None:  # set a configuration only on an unconfigured unit
            self.d.set_configuration()
        usb.util.claim_interface(self.d, 0)

    def _attach(self):
        """Drain what an interrupted transaction left, then read the calibration. Nothing is written to the outputs."""
        self._dir_set = False
        for ep in (EP_REPLY, EP_DATA):
            for _ in range(64):
                try:
                    self.d.read(ep, 512, 100)
                except timeouts():
                    break
        self._set_cal(self.read_cal())

    def send(self, m):
        """Send one message; return its reply (kind 1) or None (kind 0)."""
        self.d.write(EP_CMD, m, 5000)
        if m[6] != 1:
            return None
        for _ in range(64):
            r = bytes(self.d.read(EP_REPLY, 512, int(self.timeout * 1000)))
            if len(r) >= 12 and r[:2] == m[:2]:
                return r
        raise Error("no reply on channel %d among 64" % m[1])

    def check(self, m):
        r = self.send(m)
        if r is not None and r[11] != 2:
            raise Error("%s -> status %02x (%s)" % (m.hex(" "), r[11], r.hex(" ")))
        return r

    def _samples(self, want, deadline, poll):
        """want bytes from ep82; with poll, NI-DAQmx's poll between drains (a timed read)."""
        raw = b""
        while len(raw) < want:
            if time.time() > deadline:
                raise Error("AI: %d of %d bytes" % (len(raw), want))
            if poll:
                for m in AI_POLL:
                    self.check(m)
            # 64-byte reads (one packet each): pyusb drops a partial transfer on a timeout
            while len(raw) < want:
                try:
                    raw += bytes(self.d.read(EP_DATA, 64, int(POLL_S * 1000) if poll else 3000))
                except timeouts():
                    if not poll:
                        raise
                    break
        return raw[:want]

    # ---------------------------------------------------------------- calibration
    def read_cal(self):
        mem = b""
        for addr in range(0, CAL_SIZE, CAL_CHUNK):
            r = self.check(cal_read(addr))
            if len(r) < 14 + CAL_CHUNK or struct.unpack(">H", r[12:14])[0] != CAL_CHUNK + 2:
                raise Error("calibration read at %#x: %s" % (addr, r.hex(" ")))
            mem += r[14:14 + CAL_CHUNK]
        return mem

    def _set_cal(self, mem):
        self.cal_mem = bytes(mem)
        self.cal = parse_cal(self.cal_mem)

    # ---------------------------------------------------------------- AI
    def scale(self, word, vrange=RSE_RANGE, mode="RSE", chan=0):
        """A sample word to volts, from the unit's calibration.

        DIFF: the word unsigned, lo + (word + 0.5) * (hi - lo) / 65536 (NI-DAQmx's
        volts within a microvolt). RSE: the word signed (-32768..0),
        hi + (word + 0.5) * (hi - lo) / 32768 (within 4 mV of NI-DAQmx's: the
        exact RSE formula is not recovered).
        """
        if mode == "DIFF":
            lo, hi = self.cal["diff"][(vrange, chan)]
            return lo + (word + 0.5) * (hi - lo) / 65536
        lo, hi = self.cal["rse"][chan]
        s = word - 0x10000 if word & 0x8000 else word
        return hi + (s + 0.5) * (hi - lo) / 32768

    def _teardown(self):
        for m in AI_TEARDOWN:
            self.send(m)

    def ai_read(self, chan=0, vrange=RSE_RANGE, mode="RSE", n=1):
        """n software-timed reads of one channel in one request: ([volts], [words])."""
        check_channels([chan], vrange, mode)
        if not 1 <= n <= 0xFFFF:
            raise Error("on-demand samples: 1..65535")
        msgs = ai_ondemand_msgs([chan], vrange, mode, n)
        for m in msgs[:-1]:
            self.check(m)
        try:
            self.send(msgs[-1])
            raw = self._samples(2 * n, time.time() + 10.0, poll=False)
        except usb_errors():
            raise  # the unit is gone or wedged: no teardown to send
        except BaseException:
            self._teardown()
            raise
        self._teardown()
        words = list(struct.unpack(">%dH" % n, raw))
        return [self.scale(w, vrange, mode, chan) for w in words], words

    def timed_divisor(self, chans, rate):
        """The divisor for rate scans/s; refused above 10 kS/s aggregate or beyond 16 bits."""
        if not rate > 0:
            raise Error("timed AI: rate must be > 0")
        div = round_half_up(CLOCK / rate)
        if not 1 <= div <= MAX_DIVISOR:
            raise Error("timed AI: %g Hz needs a clock divisor of %d; 1..%d are known (%.0f Hz and up)"
                        % (rate, div, MAX_DIVISOR, CLOCK / MAX_DIVISOR))
        if CLOCK / div * len(chans) > MAX_AI_RATE:
            raise Error("timed AI: %d channels at %g Hz exceed the 6008's 10 kS/s aggregate"
                        % (len(chans), CLOCK / div))
        return div

    def ai_timed(self, chans, rate, n, vrange=RSE_RANGE, mode="RSE"):
        """A finite hardware-timed scan: n samples per channel at rate (Hz), the 24 MHz clock divided.

        Returns {chan: [volts]} and the interleaved words.
        """
        if isinstance(chans, int):
            chans = [chans]
        check_channels(chans, vrange, mode)
        if not 1 <= n <= 0xFFFFFFFF // (2 * len(chans)):
            raise Error("timed AI: bad sample count %d" % n)
        div = self.timed_divisor(chans, rate)
        for m in ai_timed_msgs(chans, vrange, mode, div, n):
            self.check(m)
        want = 2 * n * len(chans)
        try:
            raw = self._samples(want, time.time() + n * div / CLOCK + 5.0, poll=True)
        except usb_errors():
            raise
        except BaseException:
            self._teardown()
            raise
        self._teardown()
        words = list(struct.unpack(">%dH" % (want // 2), raw))
        k = len(chans)
        return {c: [self.scale(w, vrange, mode, c) for w in words[i::k]] for i, c in enumerate(chans)}, words

    # ---------------------------------------------------------------- AO
    def ao_code(self, chan, volts):
        """The AO code for volts (clamped to 0..5 V), from the unit's AO calibration."""
        lo, hi = self.cal["ao"][chan]
        v = min(AO_MAX, max(0.0, volts))
        return max(-0x800, min(0xFFF, round_half_up((v - lo) / (hi - lo) * 4095)))

    def ao_write(self, chan, volts):
        """Static AO update; returns the code written (may be below 0 at 0 V, as NI-DAQmx sends it)."""
        if chan not in self.cal["ao"]:
            raise Error("the 6008 has no AO%d" % chan)
        code = self.ao_code(chan, volts)
        self.check(ao_msg(chan, code))
        return code

    # ---------------------------------------------------------------- DIO
    def di_read(self, port=0):
        """A port's lines: port 0 has 8, port 1 has 4 (P1 reads as f0 | lines; its outputs read back)."""
        if port not in (0, 1):
            raise Error("the 6008 has ports 0 and 1")
        r = self.check(dio_read_msg(port))
        return r[14] if port == 0 else r[14] & 0x0F

    def do_write(self, value, mask=0x0F):
        """Port 1 (P1.0-3) as outputs: the lines in mask take value's, the others keep their state.

        As NI-DAQmx: port 1 is read first; on the first write since the open its
        state is latched and the lines made outputs (P0 stays input), then the
        value is written.
        """
        if not 0 <= value <= 0x0F or not 0 <= mask <= 0x0F:
            raise Error("port 1 is 4 lines")
        now = self.di_read(1)
        if not self._dir_set:
            self.check(dio_write_msg(1, now))
            self.check(dio_direction_msg(0x00, 0x0F))
            self._dir_set = True
        nxt = (now & ~mask | value & mask) & 0x0F
        self.check(dio_write_msg(1, nxt))
        return nxt
