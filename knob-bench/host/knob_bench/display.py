"""240×240 bench display.

Two sources share one canvas. A future knob build can stream the LVGL flush
as little-endian RGB565 rectangles (LV_COLOR_16_SWAP 0). Until those bytes
exist, the canvas is redrawn from the knob's existing [UI] log lines. The PNG
is masked to a circle so the square corners the GC9A01 bezel hides stay black.
"""

from __future__ import annotations

import io
import struct

from PIL import Image, ImageDraw, ImageFont

WIDTH = 240
HEIGHT = 240
MAGIC = 0x424F4E4B  # bytes on the wire are b"KNOB"
HELLO = struct.Struct("<IHHBB")
RECT = struct.Struct("<HHHHIIHH")
FLAG_KEYFRAME = 0x0001
FLAG_DROPPED = 0x0002

_ARC_BOX = (10, 10, 229, 229)
_ARC_START = 145
_ARC_SWEEP = 250
_ARC_WIDTH = 24


def rgb565_le_to_rgb(lo: int, hi: int) -> tuple[int, int, int]:
    pixel = lo | (hi << 8)
    red = ((pixel >> 11) & 0x1F) * 255 // 31
    green = ((pixel >> 5) & 0x3F) * 255 // 63
    blue = (pixel & 0x1F) * 255 // 31
    return red, green, blue


class DisplayParser:
    """Incremental reader for the knob's TCP 2341 byte stream."""

    def __init__(self) -> None:
        self._buf = bytearray()
        self.ready = False
        self.width = WIDTH
        self.height = HEIGHT

    def feed(self, data: bytes) -> list[dict]:
        self._buf.extend(data)
        frames: list[dict] = []
        while True:
            if not self.ready:
                if len(self._buf) < HELLO.size:
                    return frames
                magic, width, height, fmt, _flags = HELLO.unpack_from(self._buf)
                if magic != MAGIC or fmt != 1 or width != WIDTH or height != HEIGHT:
                    raise ValueError("display hello rejected")
                del self._buf[: HELLO.size]
                self.ready = True
                self.width = width
                self.height = height
                continue
            if len(self._buf) < RECT.size:
                return frames
            x, y, w, h, seq, t_ms, flags, _reserved = RECT.unpack_from(self._buf)
            if w > WIDTH or h > HEIGHT or w == 0 or h == 0:
                raise ValueError("display rect rejected")
            need = RECT.size + w * h * 2
            if len(self._buf) < need:
                return frames
            pixels = bytes(self._buf[RECT.size : need])
            del self._buf[:need]
            frames.append(
                {
                    "x": x,
                    "y": y,
                    "w": w,
                    "h": h,
                    "seq": seq,
                    "t_ms": t_ms,
                    "flags": flags,
                    "pixels": pixels,
                }
            )
        return frames


class Canvas:
    def __init__(self) -> None:
        self.image = Image.new("RGB", (WIDTH, HEIGHT), (0, 0, 0))
        self.source = "none"
        self.fresh = False
        self.dropped_frames = 0
        self.events: list[dict] = []
        self._event_seq = 0
        self.dial: int | None = None
        self.color = (136, 136, 136)
        self.color_known = False
        self.label = ""
        self._history: list[Image.Image] = []

    def apply_rect(
        self,
        x: int,
        y: int,
        w: int,
        h: int,
        pixels: bytes,
        seq: int,
        t_ms: int,
        flags: int,
    ) -> None:
        if len(pixels) != w * h * 2:
            raise ValueError("rect payload length")
        if flags & FLAG_DROPPED:
            self.fresh = False
            self.dropped_frames += 1
        for row in range(h):
            for col in range(w):
                px = x + col
                py = y + row
                if px < 0 or py < 0 or px >= WIDTH or py >= HEIGHT:
                    continue
                off = (row * w + col) * 2
                self.image.putpixel((px, py), rgb565_le_to_rgb(pixels[off], pixels[off + 1]))
        if flags & FLAG_KEYFRAME:
            self.fresh = True
        elif not (flags & FLAG_DROPPED):
            self.fresh = True
        self.source = "mirror"
        self._event_seq += 1
        self.events.append(
            {
                "seq": self._event_seq,
                "t_ms": t_ms,
                "x": x,
                "y": y,
                "w": w,
                "h": h,
                "keyframe": bool(flags & FLAG_KEYFRAME),
                "dropped": bool(flags & FLAG_DROPPED),
                "wire_seq": seq,
            }
        )
        if len(self.events) > 2000:
            self.events = self.events[-2000:]
        self._remember()

    def set_reconstructed(self, dial: int | None, color: tuple[int, int, int], label: str, known: bool) -> None:
        if self.source == "mirror":
            return
        self.dial = None if dial is None else max(0, min(100, dial))
        self.color = color
        self.color_known = known
        self.label = label
        self.image = _draw_dial(self.dial, self.color, self.label)
        self.source = "log" if dial is not None else "none"
        self.fresh = False
        self._remember()

    def png(self) -> bytes:
        masked = _mask_circle(self.image)
        out = io.BytesIO()
        masked.save(out, format="PNG")
        return out.getvalue()

    def gif(self, limit: int = 30) -> bytes:
        frames = self._history[-limit:] or [_mask_circle(self.image)]
        frames = [_mask_circle(frame) for frame in frames]
        out = io.BytesIO()
        head, *rest = frames
        head.save(
            out,
            format="GIF",
            save_all=True,
            append_images=rest,
            duration=80,
            loop=0,
        )
        return out.getvalue()

    def events_since(self, seq: int) -> list[dict]:
        return [event for event in self.events if event["seq"] > seq]

    def _remember(self) -> None:
        self._history.append(self.image.copy())
        if len(self._history) > 40:
            self._history = self._history[-40:]


def _draw_dial(dial: int | None, color: tuple[int, int, int], label: str) -> Image.Image:
    image = Image.new("RGB", (WIDTH, HEIGHT), (0, 0, 0))
    draw = ImageDraw.Draw(image)
    end = (_ARC_START + _ARC_SWEEP) % 360
    draw.arc(_ARC_BOX, start=_ARC_START, end=end, fill=(51, 51, 51), width=_ARC_WIDTH)
    if dial is not None and dial > 0:
        sweep = _ARC_SWEEP * dial / 100.0
        draw.arc(
            _ARC_BOX,
            start=_ARC_START,
            end=(_ARC_START + sweep) % 360,
            fill=color,
            width=_ARC_WIDTH,
        )
    font = _font(48)
    small = _font(16)
    value = "" if dial is None else str(dial)
    if value:
        box = draw.textbbox((0, 0), value, font=font)
        tw = box[2] - box[0]
        th = box[3] - box[1]
        draw.text(((WIDTH - tw) / 2, (HEIGHT - th) / 2 - 8), value, fill=(255, 255, 255), font=font)
    if label:
        shown = label.replace("\n", " ")
        box = draw.textbbox((0, 0), shown, font=small)
        tw = box[2] - box[0]
        draw.text(((WIDTH - tw) / 2, HEIGHT - 52), shown, fill=(255, 255, 255), font=small)
    return image


def _font(size: int) -> ImageFont.ImageFont:
    try:
        return ImageFont.load_default(size)
    except TypeError:
        return ImageFont.load_default()


def _mask_circle(image: Image.Image) -> Image.Image:
    mask = Image.new("L", (WIDTH, HEIGHT), 0)
    ImageDraw.Draw(mask).ellipse((0, 0, WIDTH - 1, HEIGHT - 1), fill=255)
    black = Image.new("RGB", (WIDTH, HEIGHT), (0, 0, 0))
    return Image.composite(image, black, mask)

