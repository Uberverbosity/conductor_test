import io
import json
import threading
from urllib.request import Request, urlopen

from PIL import Image

from knob_bench.commands import encode_command
from knob_bench.serial_port import open_quiet
from knob_bench.server import serve
from knob_bench.state import BenchState


def test_encode_command_matches_the_line_protocol():
    assert encode_command({"turn": -3}) == "TURN -3\n"
    assert encode_command({"click": True}) == "CLICK\n"
    assert encode_command({"touch": {"x": 12, "y": 40, "phase": "down"}}) == "TOUCH 12 40 down\n"
    for bad in ({"turn": True}, {"turn": 200}, {"touch": {"x": 300, "y": 1, "phase": "down"}}):
        try:
            encode_command(bad)
        except ValueError as exc:
            assert str(exc) == "ERR range"
        else:
            raise AssertionError(bad)


class _FakeSerial:
    def __init__(self):
        self.port = None
        self.baudrate = None
        self.timeout = None
        self.dsrdtr = None
        self.rtscts = None
        self.dtr = True
        self.rts = True
        self.dtr_at_open = None

    def open(self):
        self.dtr_at_open = self.dtr

    def setDTR(self, value):
        self.dtr = value

    def setRTS(self, value):
        self.rts = value


def test_serial_open_does_not_raise_dtr():
    handle = open_quiet("/dev/does-not-matter", 115200, factory=_FakeSerial)
    assert handle.dtr_at_open is False
    assert handle.dtr is False
    assert handle.rts is False


def _start(state: BenchState):
    httpd = serve(state, "127.0.0.1", 0)
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    host, port = httpd.server_address
    return httpd, f"http://{host}:{port}"


def _get(url: str) -> bytes:
    with urlopen(url) as response:
        return response.read()


def _post(url: str, payload: dict) -> tuple[int, dict]:
    request = Request(
        url,
        data=json.dumps(payload).encode(),
        headers={"Content-Type": "application/json"},
    )
    try:
        with urlopen(request) as response:
            return response.status, json.loads(response.read())
    except Exception as exc:
        if hasattr(exc, "code"):
            return exc.code, json.loads(exc.read())
        raise


def test_api_reads_sniffer_lines_and_knob_log():
    state = BenchState()
    httpd, base = _start(state)
    try:
        state.ingest_tap_line("# knob-bench sniffer baud=230400 gap_us=304 k2d_rx=4 d2k_rx=5")
        state.ingest_tap_line("1000 K2D 42 03 FC 01 2A 00 2A")
        state.ingest_tap_line("DROP 2")
        state.ingest_knob_line("[HELIX] READY (ownership latched)")
        state.ingest_knob_line("[UI] Slot 0 color = #112233")
        state.ingest_knob_line("[HELIX] active slot = 0")
        state.ingest_knob_line("[VOL1] assign=00 steps=40 range=80.0 step=2.00")
        state.ingest_knob_line("[UI] Dial absolute = 25")

        status = json.loads(_get(base + "/status"))
        assert status["helixReady"] is True
        assert status["activeSlot"] == 0
        assert status["dial"] == 25
        assert status["displaySource"] == "log"
        assert status["displayFresh"] is False
        assert status["sniffer"]["drops"] == 2
        assert status["colorKnown"] is True

        packets = json.loads(_get(base + "/packets?since=0"))["packets"]
        assert packets[0]["name"] == "hs_poll"
        assert packets[0]["dir"] == "K2D"

        lines = json.loads(_get(base + "/log?since=0"))["lines"]
        assert any("Dial absolute" in row["line"] for row in lines)

        page = _get(base + "/").decode()
        assert "Knob bench" in page
        image = Image.open(io.BytesIO(_get(base + "/display.png")))
        assert image.size == (240, 240)
        assert image.getpixel((0, 0)) == (0, 0, 0)

        code, body = _post(base + "/input", {"turn": 1})
        assert code == 503
        assert body["error"] == "knob-down"

        state.send_command = lambda line: "OK"
        code, body = _post(base + "/input", {"click": True})
        assert code == 200
        assert body["line"] == "CLICK"
        assert body["reply"] == "OK"
    finally:
        httpd.shutdown()
