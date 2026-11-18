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
