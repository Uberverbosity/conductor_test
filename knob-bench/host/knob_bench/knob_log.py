"""Parse the text the stable knob firmware already prints.

Those lines are the serial terminal. This module does not require any change
to that firmware. Pixel mirrors and TURN/CLICK arrive only if a later knob
build speaks the bench protocol.
"""

from __future__ import annotations

import re

from knob_bench.decode import assign_label, decode_frame

_DIAL = re.compile(r"^\[UI\] Dial absolute = (-?\d+)$")
_ACTIVE_UI = re.compile(r"^\[UI\] Active dial slot = (\d+)$")
_ACTIVE_HELIX = re.compile(r"^\[HELIX\] active slot = (\d+)$")
_COLOR = re.compile(r"^\[UI\] Slot (\d+) color = #([0-9A-Fa-f]{6})$")
_ENC = re.compile(r"^\[ENC\] delta=(-?\d+)$")
_VOL = re.compile(
    r"^\[VOL(\d+)\] assign=([0-9A-Fa-f]{2}) steps=(\d+) range=([0-9.]+) step=([0-9.]+)$"
)
_VOL_OFF = re.compile(r"^\[VOL(\d+)\] disabled$")
_HEX_LINE = re.compile(
    r"^\[(RX|TX HS|TX VOL)\] \((\d+)\): ((?:[0-9A-Fa-f]{2} ?)+)\s*$"
)


def parse_knob_line(line: str) -> dict:
    text = line.strip("\r\n")
    parsed: dict = {"line": text.strip()}
    stripped = parsed["line"]

    match = _DIAL.match(stripped)
    if match:
        parsed["dial"] = int(match.group(1))
        return parsed

    match = _ACTIVE_UI.match(stripped) or _ACTIVE_HELIX.match(stripped)
    if match:
        parsed["active_slot"] = int(match.group(1))
        return parsed

    match = _COLOR.match(stripped)
    if match:
        rgb = match.group(2)
        parsed["slot_color"] = {
            "slot": int(match.group(1)),
            "r": int(rgb[0:2], 16),
            "g": int(rgb[2:4], 16),
            "b": int(rgb[4:6], 16),
        }
        return parsed

    match = _ENC.match(stripped)
    if match:
        parsed["encoder_delta"] = int(match.group(1))
        return parsed

    if stripped == "[BTN] Encoder pressed":
        parsed["button"] = "pressed"
        return parsed

    match = _VOL.match(stripped)
    if match:
        assign = int(match.group(2), 16)
        slot = int(match.group(1)) - 1
        parsed["volume"] = {
            "slot": slot,
            "valid": True,
            "assign": assign,
            "label": assign_label(assign),
            "steps": int(match.group(3)),
            "range_db": float(match.group(4)),
            "step_db": float(match.group(5)),
        }
        return parsed

    match = _VOL_OFF.match(stripped)
    if match:
        parsed["volume"] = {"slot": int(match.group(1)) - 1, "valid": False}
        return parsed

    if stripped.startswith("[HELIX]") and (
        "handshake reset" in stripped
        or "manual resync" in stripped
        or "full resync" in stripped
    ):
        parsed["ready"] = False
        return parsed

    if stripped.startswith("[HELIX]") and "READY" in stripped:
        parsed["ready"] = True
        return parsed

    match = _HEX_LINE.match(stripped)
    if match:
        data = bytes(int(tok, 16) for tok in match.group(3).split())
        parsed["wire"] = {
            "channel": match.group(1),
            "claimed_len": int(match.group(2)),
            "decoded": decode_frame(data),
        }
    return parsed
