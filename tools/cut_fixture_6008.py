#!/usr/bin/env python3
"""Cut the NI USB-6008 test fixture from USBPcap captures of NI-DAQmx.

usage: cut_fixture_6008.py OUT.json cal=PCAP:DEV steps=PCAP:DEV:PROBELOG ... [--drop NAME]

The 6008 needs no firmware and has no FX2 load, so the fixture keeps, in
capture order, the messages on bulk ep01, the replies on ep81 and the
samples on ep82 (the capture reader and the MARK cutting are
cut_fixture.py's). `cal` keeps, from a plug-in capture, only NI-DAQmx's read
of the unit's 512-byte calibration memory (subsystem 04, 32 bytes at a
time) and its replies: the unit's own data. The identity queries, DAQmx's
AO zeroing and DIO direction reset at plug-in are left out (the driver
never sends them). `steps` sections are cut by the probe's MARK lines, as
for the 621x.

From the probe log only the volts NI-DAQmx returned are kept, per section
(`daqmx`: the AI reads' values, and a timed read's first values and mean),
for the scaling tests; nothing else of the log (no host or chassis names).
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from cut_fixture import by_marks, marks, packets  # noqa: E402

CAL_READ = "140204"  # verb 14, family 02, subsystem 04 (bytes 7-9 of a message)


def events(path, dev):
    """(time, event) for one device's bulk traffic, in capture order."""
    out = []
    for p in packets(path):
        if p["dev"] != dev or p["xfer"] != 3 or not p["payload"]:
            continue
        pl, ep = p["payload"], p["ep"]
        if ep == 0x01:
            out.append((p["t"], {"out": "ep01", "hex": pl.hex()}))
        elif ep in (0x81, 0x82):
            out.append((p["t"], {"in": "ep%02x" % ep, "hex": pl.hex()}))
    return out


def cal_section(evs):
    """The calibration-memory reads (32 bytes each) and the reply to each."""
    out, keep = [], False
    for _, ev in evs:
        h = ev["hex"]
        if ev.get("out") == "ep01":
            keep = h[14:20] == CAL_READ and h[32:36] == "0020"
            if keep:
                out.append(ev)
        elif ev.get("in") == "ep81" and keep:
            out.append(ev)
            keep = False
    return out


def daqmx_values(probelog):
    """{step: {"ai": [[volts]...], "timed": [{"first": [...], "mean": v}]}} from the probe log."""
    res, step = {}, None
    for line in open(probelog):
        if " MARK " in line:
            step = line.split(" MARK ", 1)[1].strip()
            continue
        if step is None:
            continue
        if " AI " in line and " v=" in line:
            vals = [float(x) for x in line.split(" v=", 1)[1].split(",")]
            res.setdefault(step, {}).setdefault("ai", []).append(vals)
        elif " TIMED " in line and " first=" in line:
            first = line.split(" first=", 1)[1].split()[0]
            mean = float(line.split(" mean=", 1)[1].split()[0])
            res.setdefault(step, {}).setdefault("timed", []).append(
                {"first": [float(x) for x in first.split(",")], "mean": mean})
    return res


def main():
    drop = set()
    while "--drop" in sys.argv:
        i = sys.argv.index("--drop")
        drop.add(sys.argv[i + 1])
        del sys.argv[i:i + 2]
    if len(sys.argv) < 3:
        raise SystemExit(__doc__)
    fixture = {"about": "NI-DAQmx 20 and a USB-6008, cut by tools/cut_fixture_6008.py; "
                        "daqmx: the volts NI-DAQmx returned in each step",
               "sections": {}, "daqmx": {}}
    for arg in sys.argv[2:]:
        kind, spec = arg.split("=", 1)
        parts = spec.split(":")
        evs = events(parts[0], int(parts[1]))
        if kind == "steps":
            fixture["sections"].update(by_marks(evs, marks(parts[2])))
            fixture["daqmx"].update(daqmx_values(parts[2]))
        elif kind == "cal":
            fixture["sections"]["cal"] = cal_section(evs)
        else:
            raise SystemExit("unknown capture kind %s" % kind)
    for name in drop:
        fixture["sections"].pop(name, None)
        fixture["daqmx"].pop(name, None)
    fixture["daqmx"] = {k: v for k, v in fixture["daqmx"].items() if k in fixture["sections"]}
    with open(sys.argv[1], "w") as f:
        json.dump(fixture, f, indent=1)
        f.write("\n")
    for k, v in fixture["sections"].items():
        print("%-50s %d events" % (k, len(v)))


if __name__ == "__main__":
    main()
