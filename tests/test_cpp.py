"""The C++ driver (niusb/, built natively) against the NI-DAQmx capture, the
Python reference and the fake."""
import os
import shutil
import subprocess

import pytest

import replay

HERE = os.path.dirname(os.path.abspath(__file__))
BUILD = os.path.join(HERE, "cpp", "build")

pytestmark = pytest.mark.skipif(shutil.which("make") is None or shutil.which(os.environ.get("CXX", "c++")) is None,
                                reason="no native C++ toolchain")


@pytest.fixture(scope="module")
def tools():
    subprocess.run(["make", "-C", os.path.join(HERE, "cpp")], check=True, capture_output=True)
    return BUILD


def run_replay(tools, tmp_path, names, calls):
    path = tmp_path / "script.txt"
    path.write_text(replay.script(replay.fixture(), names, calls))
    out = subprocess.run([os.path.join(tools, "niusb_replay"), str(path)], check=True, capture_output=True, text=True)
    return out.stdout.splitlines()


@pytest.mark.parametrize("case,names,calls", replay.CASES, ids=[c[0] for c in replay.CASES])
def test_requests_match_daqmx_and_python(tools, tmp_path, case, names, calls):
    sections = replay.fixture()
    lines = run_replay(tools, tmp_path, names, calls)
    assert not [x for x in lines if x.startswith("error")], lines
    got = replay.requests(lines)
    py, _ = replay.run_python(sections, names, calls)
    assert got == py, "C++ and the Python reference differ"
    if case == "do":
        got.remove("ep01 " + replay.DO_SKIPPED)
    assert got == replay.expected(sections, names)


def test_results_match_python(tools, tmp_path):
    sections = replay.fixture()
    lines = run_replay(tools, tmp_path, ["ao0 0V 1V 0V"], replay.AO_STEPS)
    _, py = replay.run_python(sections, ["ao0 0V 1V 0V"], replay.AO_STEPS)
    cpp_ai = [[float(v) for v in x.split()[2:]] for x in lines if x.startswith("result ai")]
    cpp_ao = [int(x.split()[2]) for x in lines if x.startswith("result ao")]
    assert cpp_ao == [r[1] for r in py if r[0] == "ao"]
    for got, want in zip(cpp_ai, [r[1] for r in py if r[0] == "ai"]):
        assert got == pytest.approx(want, abs=1e-6)


def test_fake(tools):
    out = subprocess.run([os.path.join(tools, "niusb_fake")], capture_output=True, text=True)
    assert out.returncode == 0, out.stdout
    assert "FAIL" not in out.stdout


def test_firmware_sums(tools, tmp_path):
    """The C++ SHA-256 agrees with hashlib, through the refusal message."""
    import hashlib
    names = ["usb621x.cfg", "usb621x0.cfg", "usb621x1.cfg", "SlicFpga.cfg", "STC3Lite.cfg"]
    for n in names:
        (tmp_path / n).write_bytes(b"abc" * 9000)
    path = tmp_path / "script.txt"
    path.write_text("")
    # niusb_fake has no firmware entry point; the sum is checked through the replay tool's sibling:
    src = os.path.join(HERE, "cpp", "sha_check.cpp")
    exe = os.path.join(BUILD, "sha_check")
    subprocess.run(["make", "-C", os.path.join(HERE, "cpp"), "build/sha_check"], check=True, capture_output=True)
    out = subprocess.run([exe, str(tmp_path)], capture_output=True, text=True)
    assert out.returncode == 1
    assert "usb621x.cfg" in out.stdout
    assert hashlib.sha256(b"abc" * 9000).hexdigest() in out.stdout
    assert os.path.exists(src)
