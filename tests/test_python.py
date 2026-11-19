"""The Python reference (python/ni62xx.py) against the NI-DAQmx capture."""
import pytest

import replay


@pytest.mark.parametrize("case,names,calls", replay.CASES, ids=[c[0] for c in replay.CASES])
def test_requests_match_daqmx(case, names, calls):
    sections = replay.fixture()
    want = replay.expected(sections, names)
    got, _ = replay.run_python(sections, names, calls)
    if case == "do":
        assert got.count("ep01 " + replay.DO_SKIPPED) == 2  # the writes of 0
        got.remove("ep01 " + replay.DO_SKIPPED)  # the first one: DAQmx skips it
    assert got == want


def test_scaling_matches_daqmx():
    """AI1 through the AO0 loopback, as NI-DAQmx scaled the same codes."""
    sections = replay.fixture()
    _, results = replay.run_python(sections, ["ao0 0V 1V 0V"], replay.AO_STEPS)
    ai = [r[1] for r in results if r[0] == "ai"]
    daqmx = [[-0.000520486, -0.000191441, -0.000849531], [0.999777, 0.99879, 0.999777],
             [-0.000520486, -0.000849531, -0.000849531]]
    for got, want in zip(ai, daqmx):
        assert got == pytest.approx(want, abs=2e-5)
    assert [r[1] for r in results if r[0] == "ao"] == [1, 3216, 1]


def test_firmware_sums_refuse_a_changed_file(tmp_path):
    import ni62xx
    for name in ni62xx.FIRMWARE_FILES:
        (tmp_path / name).write_bytes(b"\0" * 20000)
    with pytest.raises(ni62xx.Error, match="usb621x.cfg"):
        ni62xx.load_firmware(str(tmp_path))


# ------------------------------------------------------------------ USB-6008
import replay6008  # noqa: E402

# DAQmx's own volts against the reference's scaling of the same words: DIFF is
# reproduced (lo + (word + 0.5) * span / 65536), RSE only approximately.
DIFF_TOL, RSE_TOL = 0.5e-3, 5e-3


@pytest.mark.parametrize("case,names,calls", replay6008.CASES, ids=[c[0] for c in replay6008.CASES])
def test_6008_requests_match_daqmx(case, names, calls):
    sections = replay6008.fixture()["sections"]
    got, _ = replay6008.run_python(sections, names, calls)
    assert got == replay6008.expected(sections, names)


def test_6008_scaling_matches_daqmx():
    fx = replay6008.fixture()
    checked = {"RSE": 0, "DIFF": 0, "timed": 0}
    for case, names, calls in replay6008.CASES:
        daqmx = fx["daqmx"].get(names[0], {})
        _, results = replay6008.run_python(fx["sections"], names, calls)
        ai_calls = [c for c in calls if c[0] == "ai"]
        ai_results = [r[1] for r in results if r[0] == "ai"]
        assert len(ai_results) == len(daqmx.get("ai", []))
        for c, got, want in zip(ai_calls, ai_results, daqmx.get("ai", [])):
            tol = DIFF_TOL if c[3] == "DIFF" else RSE_TOL
            assert got == pytest.approx(want, abs=tol), (case, c)
            checked[c[3]] += len(want)
        for got, want in zip([r[1] for r in results if r[0] == "timed"], daqmx.get("timed", [])):
            # DAQmx's timed values are logged to 5 digits: its first six of the
            # first channel, and the mean over every channel
            assert got[0][:6] == pytest.approx(want["first"], abs=RSE_TOL), case
            allv = [v for ch in got for v in ch]
            assert sum(allv) / len(allv) == pytest.approx(want["mean"], abs=RSE_TOL), case
            checked["timed"] += 1
    assert checked["DIFF"] == 9 and checked["RSE"] >= 40 and checked["timed"] == 2, checked


def test_6008_ao_codes_equal_daqmx():
    fx = replay6008.fixture()
    for case, names, calls in replay6008.CASES:
        if not case.startswith("ao"):
            continue
        sent = [ev["hex"] for ev in fx["sections"][names[0]] if ev.get("out") == "ep01" and ev["hex"][14:20] == "0e0221"]
        daqmx = [int.from_bytes(bytes.fromhex(h[24:28]), "big", signed=True) for h in sent]
        _, results = replay6008.run_python(fx["sections"], names, calls)
        assert [r[1] for r in results if r[0] == "ao"] == daqmx
    assert daqmx == [-5, 809, 2029, 3655, -5]  # AO1: 0 V is below code 0, sent as DAQmx sends it


def test_6008_open_reads_only():
    """The open reads the calibration and writes no AO and no DIO direction or value."""
    sections = replay6008.fixture()["sections"]
    got, results = replay6008.run_python(sections, ["cal"], [("open",)])
    assert all(x[14:20] == "140204" for x in (g.split()[1] for g in got)), got
    assert bytes.fromhex(results[0][1])[0x1FC:0x200].hex() == "019cdc74"  # the unit's serial, in its memory


def test_6008_refusals():
    import ni6008
    d = ni6008.USB6008.adopt(None, replay6008.cal_memory(replay6008.fixture()["sections"]))
    for call in (lambda: d.ai_read(0, 5.0, "RSE"),            # RSE is +-10 V only
                 lambda: d.ai_read(4, 10.0, "DIFF"),           # DIFF on channels 0-3
                 lambda: d.ai_read(0, 10.0, "NRSE"),
                 lambda: d.ai_read(8, 10.0, "RSE"),
                 lambda: d.ai_timed([0, 1], 6000, 10),         # 12 kS/s aggregate
                 lambda: d.ai_timed([0], 100, 10),             # a divisor beyond 16 bits
                 lambda: d.ao_write(2, 1.0),
                 lambda: d.do_write(16)):
        with pytest.raises(ni6008.Error):
            call()
    assert d.ao_code(0, 7.0) == d.ao_code(0, 5.0) and d.ao_code(1, -1.0) == d.ao_code(1, 0.0)
