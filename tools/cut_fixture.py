#!/usr/bin/env python3
"""Cut the NI USB-621x test fixture from USBPcap captures of NI-DAQmx.

usage: cut_fixture.py OUT.json fx2=PCAP:DEV bringup=PCAP:DEV steps=PCAP:DEV:PROBELOG ...

Each capture (USBPcap, linktype 249) is read with the standard library only.
The fixture keeps what our driver must reproduce or answer: the messages on
bulk ep01, the replies on ep81, the samples on ep88, and the FX2 vendor
control requests. NI's own bytes (the FX2 code in the control transfers, the
FPGA bitstreams on ep06) are replaced by their length and SHA-256, so the
fixture carries no vendor code (geecs-epics decision 23). NI-DAQmx's 50 ms
status poll on channel 0 (`0000000c0008010100000000`) and its replies are
dropped: our driver does not send it.

`steps` sections are cut by the probe's MARK lines (`HH:MM:SS.fff MARK
name`, the capturing PC's local time; run this in the same time zone). The
tail after "end" joins the last step; `--drop NAME` leaves a step out.
"""
import datetime
import hashlib
import json
import struct
import sys

POLL = bytes.fromhex("0000000c0008010100000000")
KEEP_HEX = 2  # control data this short is not vendor code (CPUCS, the B0 reply)


def packets(path):
    with open(path, "rb") as f:
        head = f.read(24)
        if struct.unpack("<I", head[:4])[0] != 0xA1B2C3D4 or struct.unpack("<I", head[20:24])[0] != 249:
            raise SystemExit("%s: not a USBPcap capture" % path)
        while True:
            ph = f.read(16)
            if len(ph) < 16:
                return
            ts, tus, incl, _ = struct.unpack("<IIII", ph)
            data = f.read(incl)
            hl, _irp, _st, _fn, info, _bus, dev, ep, xfer, _dl = struct.unpack("<HQIHBHHBBI", data[:27])
            stage = data[27] if xfer == 2 and hl >= 28 else None
            yield dict(t=ts + tus / 1e6, info=info, dev=dev, ep=ep, xfer=xfer, stage=stage, payload=data[hl:])


def digest(b):
    return {"len": len(b), "sha256": hashlib.sha256(b).hexdigest()}


def events(path, dev):
    """(time, event) for one device, in capture order, polls dropped."""
    out, pending_in, drop_reply = [], None, False
    for p in packets(path):
        if p["dev"] != dev:
            continue
        pl, t = p["payload"], p["t"]
        if p["xfer"] == 2:  # control
            if p["stage"] == 0 and len(pl) >= 8:
                rt, req, val, idx, n = struct.unpack("<BBHHH", pl[:8])
                if rt & 0x60 != 0x40:  # standard requests (descriptors) are the host stack's
                    pending_in = None
                    continue
                ev = {"ctrl": "out" if rt == 0x40 else "in", "req": req, "value": val, "index": idx, "length": n}
                if rt == 0x40:
                    data = pl[8:]
                    ev.update({"data": data.hex()} if len(data) <= KEEP_HEX else digest(data))
                    out.append((t, ev))
                else:
                    pending_in = ev
            elif p["stage"] == 3 and pending_in is not None and p["info"] & 1:
                ev = dict(pending_in)
                ev.update({"data": pl.hex()} if len(pl) <= KEEP_HEX else digest(pl))
                out.append((t, ev))
                pending_in = None
            continue
        if p["xfer"] != 3 or not pl:
            continue
        ep = p["ep"]
        if ep == 0x01:
            if pl == POLL:
                drop_reply = True
                continue
            out.append((t, {"out": "ep01", "hex": pl.hex()}))
        elif ep == 0x81:
            if drop_reply and pl[:2] == b"\0\0":
                drop_reply = False
                continue
            out.append((t, {"in": "ep81", "hex": pl.hex()}))
        elif ep == 0x88:
            out.append((t, {"in": "ep88", "hex": pl.hex()}))
        elif ep == 0x06:
            ev = {"out": "ep06"}
            ev.update(digest(pl))
            if out and out[-1][1].get("out") == "ep06":  # one stream: merge the 512-byte writes
                prev = out[-1][1]
                prev.setdefault("_chunks", []).append(pl)
                continue
            ev["_chunks"] = [pl]
            out.append((t, ev))
        else:
            out.append((t, {"other": "ep%02x" % ep, **digest(pl)}))
    for _, ev in out:  # a merged stream's digest is over the whole stream
        if "_chunks" in ev:
            whole = b"".join(ev.pop("_chunks"))
            ev.update(digest(whole))
            ev["writes"] = -(-len(whole) // 512)
    return out


def marks(probelog):
    day = None
    res = []
    for line in open(probelog):
        if " MARK " in line:
            hhmmss, name = line[:12], line.split(" MARK ", 1)[1].strip()
            res.append((hhmmss, name))
    return res


def by_marks(evs, mk):
    if not evs:
        return {}
    day = datetime.datetime.fromtimestamp(evs[0][0]).date()
    edges = [(datetime.datetime.combine(day, datetime.datetime.strptime(h, "%H:%M:%S.%f").time()).timestamp(), n)
             for h, n in mk]
    sections, last = {}, None
    for i, (t0, name) in enumerate(edges):
        t1 = edges[i + 1][0] if i + 1 < len(edges) else float("inf")
        sec = [ev for t, ev in evs if t0 <= t < t1]
        if name == "end" and last is not None:
            # The probe logs "end" when its last call returns; the capture's
            # clock puts that step's tail just after it.
            sections[last].extend(sec)
        elif sec and not name.startswith("start"):
            sections[name] = sec
            last = name
    return sections


def main():
    drop = set()
    while "--drop" in sys.argv:
        i = sys.argv.index("--drop")
        drop.add(sys.argv[i + 1])
        del sys.argv[i:i + 2]
    if len(sys.argv) < 3:
        raise SystemExit(__doc__)
    fixture = {"about": "NI-DAQmx 20 and a USB-6211, cut by tools/cut_fixture.py; NI's bytes as length and SHA-256",
               "sections": {}}
    for arg in sys.argv[2:]:
        kind, spec = arg.split("=", 1)
        parts = spec.split(":")
        evs = events(parts[0], int(parts[1]))
        if kind == "steps":
            fixture["sections"].update(by_marks(evs, marks(parts[2])))
        else:
            fixture["sections"][kind] = [ev for _, ev in evs]
    for name in drop:
        fixture["sections"].pop(name, None)
    with open(sys.argv[1], "w") as f:
        json.dump(fixture, f, indent=1)
        f.write("\n")
    for k, v in fixture["sections"].items():
        print("%-28s %d events" % (k, len(v)))


if __name__ == "__main__":
    main()
