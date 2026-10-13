import io
import struct

from PIL import Image

from knob_bench.display import HELLO, MAGIC, RECT, Canvas, DisplayParser


def test_reconstruction_changes_with_dial_and_hides_corners():
    canvas = Canvas()
    canvas.set_reconstructed(0, (255, 0, 0), "MASTER VOLUME", True)
    low = Image.open(io.BytesIO(canvas.png()))
    canvas.set_reconstructed(100, (255, 0, 0), "MASTER VOLUME", True)
    high = Image.open(io.BytesIO(canvas.png()))
    assert low.size == (240, 240)
    assert low.getpixel((0, 0)) == (0, 0, 0)
    assert high.getpixel((0, 0)) == (0, 0, 0)
    assert low.tobytes() != high.tobytes()


def test_mirror_rect_is_masked_at_the_corner():
    canvas = Canvas()
    white = b"\xff\xff"
    canvas.apply_rect(0, 0, 1, 1, white, seq=1, t_ms=0, flags=0)
    canvas.apply_rect(120, 120, 1, 1, white, seq=2, t_ms=1, flags=1)
    image = Image.open(io.BytesIO(canvas.png()))
    assert image.getpixel((0, 0)) == (0, 0, 0)
    assert image.getpixel((120, 120)) == (255, 255, 255)
    assert canvas.fresh is True
    assert canvas.source == "mirror"
    assert canvas.events_since(1)[0]["keyframe"] is True


def test_display_parser_accepts_a_split_hello_and_rect():
    parser = DisplayParser()
    hello = HELLO.pack(MAGIC, 240, 240, 1, 0)
    rect = RECT.pack(10, 10, 1, 1, 7, 99, 0, 0) + b"\x00\xf8"
    assert parser.feed(hello[:4]) == []
    frames = parser.feed(hello[4:] + rect)
    assert parser.ready is True
    assert frames[0]["seq"] == 7
    assert frames[0]["pixels"] == b"\x00\xf8"


def test_bad_hello_is_rejected():
    parser = DisplayParser()
    try:
        parser.feed(struct.pack("<IHHBB", 0, 240, 240, 1, 0))
    except ValueError as exc:
        assert "hello" in str(exc)
    else:
        raise AssertionError("expected rejection")
