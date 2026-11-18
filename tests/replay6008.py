"""The USB-6008's DAQmx fixture as replay scripts, and the Python reference run on them.

Each case names fixture sections and the driver calls NI-DAQmx's probe made
in them. A reply or a sample transfer is served only once as many ep01
messages have been sent as preceded it in the capture: a read before then
times out, as on the unit. So the open's drain of stale replies finds
nothing, and a timed read polls (13 02 03, 10 02 03) until the samples are
due, as many times as NI-DAQmx did. Both drivers must send exactly the
fixture's ep01 messages (test_python.py, test_cpp.py).
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
FIXTURE = os.path.join(HERE, "fixtures", "daqmx-6008.json")
sys.path.insert(0, os.path.join(HERE, "..", "python"))

import ni6008  # noqa: E402

# A call: ("open",), ("ai", chan, range, mode, n), ("timed", chans, rate, n,
# range, mode), ("ao", chan, volts), ("do", value), ("di", port).
AO_VOLTS = (0.0, 1.0, 2.5, 4.5, 0.0)
CASES = [
    ("open", ["cal"], [("open",)]),
    ("ai0-x1", ["ai0 RSE 10V x1"], [("ai", 0, 10.0, "RSE", 1)]),
    ("ai0-x10", ["ai0 RSE 10V x10"], [("ai", 0, 10.0, "RSE", 10)]),
    ("ai1", ["ai1 RSE 10V x3"], [("ai", 1, 10.0, "RSE", 3)]),
    ("ai7", ["ai7 RSE 10V x3"], [("ai", 7, 10.0, "RSE", 3)]),
    ("diff-20V", ["ai0 DIFF 20V x3"], [("ai", 0, 20.0, "DIFF", 3)]),
    ("diff-1V", ["ai0 DIFF 1V x3"], [("ai", 0, 1.0, "DIFF", 3)]),
    ("diff-5V", ["ai0 DIFF 5V x3"], [("ai", 0, 5.0, "DIFF", 3)]),
    ("ai0-again", ["ai0 RSE 10V again x3"], [("ai", 0, 10.0, "RSE", 3)]),
    ("di0", ["di port0"], [("di", 0)]),
    ("di1", ["di port1"], [("di", 1)]),
    ("timed-1ch", ["timed ai0 1kHz x1000"], [("timed", [0], 1000, 1000, 10.0, "RSE")]),
    ("timed-2ch", ["timed ai0:1 5kHz x500"], [("timed", [0, 1], 5000, 500, 10.0, "RSE")]),
    ("ai-before", ["ai0 ai1 before"], [("ai", 0, 10.0, "RSE", 3), ("ai", 1, 10.0, "RSE", 3)]),
    ("di-before", ["di port0 before"], [("di", 0)]),
    ("ao0", ["ao0 0 1 2.5 4.5 0"], [c for v in AO_VOLTS for c in (("ao", 0, v), ("ai", 0, 10.0, "RSE", 3))]),
    ("ao1", ["ao1 0 1 2.5 4.5 0"], [c for v in AO_VOLTS for c in (("ao", 1, v), ("ai", 1, 10.0, "RSE", 3))]),
    ("do", ["do port1 0x0 0x1 0x0 0xF 0x0"], [c for v in (0, 1, 0, 15, 0) for c in (("do", v), ("di", 0))]),
]
# Not replayed: "do port0 line0 ..." (P0 as an output is not offered) and the
# counter step (not cut).


def fixture():
    with open(FIXTURE) as f:
        return json.load(f)


def cal_memory(sections):
    """The unit's 512-byte calibration memory as NI-DAQmx read it at plug-in."""
    out, events = b"", sections["cal"]
    for i, ev in enumerate(events):
        if ev.get("out") == "ep01":
            r = bytes.fromhex(events[i + 1]["hex"])
            out += r[14:14 + ni6008.CAL_CHUNK]
    return out


def gated(sections, names):
    """[(ep, writes before it, hex)] for every reply and sample transfer, in order."""
    out, writes = [], 0
    for n in names:
        for ev in sections[n]:
            if ev.get("out") == "ep01":
                writes += 1
            elif "in" in ev:
                out.append((ev["in"], writes, ev["hex"]))
    return out


def expected(sections, names):
    return ["ep01 " + ev["hex"] for n in names for ev in sections[n] if ev.get("out") == "ep01"]


def script(sections, names, calls):
    out = ["cal " + cal_memory(sections).hex()]
    out += ["%s %d %s" % x for x in gated(sections, names)]
    for c in calls:
        out.append(" ".join(",".join(map(str, x)) if isinstance(x, list) else str(x) for x in c))
    return "\n".join(out) + "\n"


def requests(lines):
    return [x for x in lines if x.startswith("ep01 ")]


# ------------------------------------------------------------- Python reference
class ReplayDev:
    """A pyusb-shaped device serving the gated replies and logging the requests."""

    def __init__(self, events):
        self.q = {"ep81": [], "ep82": []}
        for ep, after, h in events:
            self.q[ep].append([after, bytes.fromhex(h)])
        self.writes, self.lines = 0, []

    def write(self, ep, data, timeout=None):
        self.lines.append("ep%02x %s" % (ep, bytes(data).hex()))
        self.writes += 1
        return len(data)

    def read(self, ep, size, timeout=None):
        q = self.q["ep%02x" % ep]
        if not q or q[0][0] > self.writes:
            raise TimeoutError("replay: nothing due on ep%02x" % ep)
        x = q[0][1]
        out, rest = x[:size], x[size:]
        if rest:
            q[0][1] = rest
        else:
            q.pop(0)
        return out


def run_python(sections, names, calls):
    dev = ReplayDev(gated(sections, names))
    results = []
    if calls[0][0] == "open":
        d = ni6008.USB6008.__new__(ni6008.USB6008)
        d.d, d.timeout, d.serial = dev, 10.0, None
        d._attach()
        results.append(("open", d.cal_mem.hex()))
        calls = calls[1:]
    else:
        d = ni6008.USB6008.adopt(dev, cal_memory(sections))
    for c in calls:
        if c[0] == "ai":
            v, _ = d.ai_read(c[1], c[2], c[3], c[4])
            results.append(("ai", v))
        elif c[0] == "timed":
            v, words = d.ai_timed(c[1], c[2], c[3], c[4], c[5])
            results.append(("timed", [v[ch] for ch in c[1]]))
        elif c[0] == "ao":
            results.append(("ao", d.ao_write(c[1], c[2])))
        elif c[0] == "do":
            results.append(("do", d.do_write(c[1])))
        elif c[0] == "di":
            results.append(("di", d.di_read(c[1])))
    return dev.lines, results
