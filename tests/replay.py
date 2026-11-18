"""The DAQmx fixture as replay scripts, and the Python reference run on them.

Each case names a fixture section and the driver calls that NI-DAQmx's
probe made in it. From the section come the unit's replies (queued in
order) and the expected requests: the FX2 control requests, the ep01
messages and the ep06 streams. Both drivers must make exactly those
requests (test_cpp.py, test_python.py).
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
FIXTURE = os.path.join(HERE, "fixtures", "daqmx-621x.json")
sys.path.insert(0, os.path.join(HERE, "..", "python"))

import ni62xx  # noqa: E402

# (case, sections, calls). A call: ("ai", chan, range, mode, n), ("timed",
# chans, rate, n, range, mode), ("ao", chan, volts), ("do", value), ("pfi",).
FW_SIZES = (1284, 16384, 10438, 3601, 78422, 283776)
AO_STEPS = [c for v in (0.0, 1.0, 0.0) for c in (("ao", 0, v), ("ai", 1, 10.0, "RSE", 3))]
CASES = [
    ("open", ["fx2", "bringup"], [("open",)]),
    ("ai-10V", ["ai-10V"], [("ai", 0, 10.0, "RSE", 10)]),
    ("ai-0.2V", ["ai-0.2V"], [("ai", 0, 0.2, "RSE", 10)]),
    ("ai-10V-again", ["ai-10V-again"], [("ai", 0, 10.0, "RSE", 10)]),
    ("di", ["di"], [("pfi",)]),
    ("ao0", ["ao0 0V 1V 0V"], AO_STEPS),
    ("do", ["do port1 0x0 0x1 0x0"], [c for v in (0, 1, 0) for c in (("do", v), ("pfi",))]),
    ("timed-1k", ["timed ai0 1kHz x1000"], [("timed", [0], 1000, 1000, 10.0, "RSE")]),
    ("timed-2ch", ["timed ai0:1 10kHz x200"], [("timed", [0, 1], 10000, 200, 10.0, "RSE")]),
]
# NI-DAQmx's first write of a DO task whose lines already read 0 sends only
# the line setup (0a 28) and skips the value (09 28); the drivers always send
# both. That one message is the known difference.
DO_SKIPPED = ni62xx.msg(1, 0x09, 0x28, 3, bytes.fromhex("000000f0") + (0).to_bytes(4, "big")).hex()
OK_CH1 = "0001000c0008010000000002"  # the reply the drivers' extra message gets in the replay


def replies(sections, names, ep):
    out = [ev["hex"] for n in names for ev in sections[n] if ev.get("in") == ep]
    if ep == "ep81" and names == ["do port1 0x0 0x1 0x0"]:
        out.insert(1, OK_CH1)
    return out


def fixture():
    with open(FIXTURE) as f:
        return json.load(f)["sections"]


def eeprom(sections):
    """The unit's EEPROM as NI-DAQmx read it at bring-up."""
    out = b""
    events = sections["bringup"]
    for i, ev in enumerate(events):
        h = ev.get("hex", "")
        if ev.get("out") == "ep01" and h[14:20] == "080305" and len(h) == 40:
            n = int(h[32:40], 16)
            out += bytes.fromhex(events[i + 1]["hex"])[14:14 + n]
    return out


def expected(sections, names):
    lines = []
    for name in names:
        for ev in sections[name]:
            if "ctrl" in ev:
                lines.append("ctrl %s %d %d %d %d" % (ev["ctrl"], ev["req"], ev["value"], ev["index"], ev["length"]))
            elif ev.get("out") == "ep01":
                lines.append("ep01 " + ev["hex"])
            elif ev.get("out") == "ep06":
                lines.append("ep06 %d" % ev["len"])
    return lines


def script(sections, names, calls):
    out = []
    if names[0] == "fx2":
        out.append("fw " + " ".join(str(n) for n in FW_SIZES))
    else:
        out.append("eeprom " + eeprom(sections).hex())
    for ep in ("ep81", "ep88"):
        out += ["%s %s" % (ep, h) for h in replies(sections, names, ep)]
    for c in calls:
        out.append(" ".join(",".join(map(str, x)) if isinstance(x, list) else str(x) for x in c))
    return "\n".join(out) + "\n"


def requests(lines):
    """The request lines of a driver's output, without results."""
    return [x for x in lines if x.split(" ", 1)[0] in ("ctrl", "ep01", "ep06")]


# ------------------------------------------------------------- Python reference
class ReplayDev:
    """A pyusb-shaped device serving the queued replies and logging requests."""

    def __init__(self, ep81, ep88):
        self.ep81, self.ep88, self.lines, self.fx2, self.streamed = list(ep81), list(ep88), [], {}, 0

    def _flush(self):
        if self.streamed:
            self.lines.append("ep06 %d" % self.streamed)
        self.streamed = 0

    def ctrl_transfer(self, rt, req, value, index, data_or_len, timeout=None):
        self._flush()
        if rt == 0x40:
            self.lines.append("ctrl out %d %d %d %d" % (req, value, index, len(data_or_len)))
            self.fx2[(req, value)] = bytes(data_or_len)
            return len(data_or_len)
        self.lines.append("ctrl in %d %d %d %d" % (req, value, index, data_or_len))
        return b"\x70\x72" if req == 0xB0 else self.fx2.get((req, value), bytes(data_or_len))

    def write(self, ep, data, timeout=None):
        if ep == 0x06:
            self.streamed += len(data)
            return len(data)
        self._flush()
        self.lines.append("ep%02x %s" % (ep, bytes(data).hex()))
        return len(data)

    def read(self, ep, size, timeout=None):
        self._flush()
        q = self.ep81 if ep == 0x81 else self.ep88
        if not q:
            raise TimeoutError("replay: nothing left on ep%02x" % ep)
        x = q[0]
        out, rest = x[:size], x[size:]
        if rest:
            q[0] = rest
        else:
            q.pop(0)
        return out


def run_python(sections, names, calls):
    dev = ReplayDev(*([bytes.fromhex(h) for h in replies(sections, names, ep)] for ep in ("ep81", "ep88")))
    results = []
    if names[0] == "fx2":
        d = ni62xx.USB621x.__new__(ni62xx.USB621x)
        d.timeout, d.model, d.do_value = 10.0, "USB-6211", 0
        d.fw = {k: bytes((j * 7 + i) & 0xFF for j in range(n))
                for i, (k, n) in enumerate(zip(("loader", "main16k", "bank0", "bank1", "fpga0", "fpga1"), FW_SIZES))}
        ni62xx.usb = type("U", (), {"util": type("V", (), {"dispose_resources": staticmethod(lambda d: None)})})
        d._fx2(dev)
        d.d = dev
        d._bringup()
        results.append(("open", d.eeprom[:2].hex()))
    else:
        d = ni62xx.USB621x.adopt(dev, eeprom(sections))
        for c in calls:
            if c[0] == "ai":
                v, codes = d.ai_read(c[1], c[2], c[3], c[4])
                results.append(("ai", v))
            elif c[0] == "timed":
                v, codes = d.ai_timed(c[1], c[2], c[3], c[4], c[5])
                results.append(("timed", codes))
            elif c[0] == "ao":
                results.append(("ao", d.ao_write(c[1], c[2])))
            elif c[0] == "do":
                d.do_write(c[1])
            elif c[0] == "pfi":
                results.append(("pfi", d.pfi_read()))
    dev._flush()
    return dev.lines, results
