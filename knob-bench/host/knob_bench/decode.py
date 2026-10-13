"""Decode bytes heard on the knob/DSP UART.

Indexing matches helix_protocol.cpp: a 0x42 frame's type is byte 2.
Volume writes are the exception. The knob sends them as a 0x42 0x06 wake,
a 600 µs gap, then an F9 body that does not start with 0x42.
"""

from __future__ import annotations

import re

SLOT_BASE0 = 22
SLOT_STRIDE = 10
NUM_SLOTS = 4
VOL_MIN = 0x30

_ASSIGN_LABELS = {
    0x00: "MASTER\nVOLUME",
    0x01: "SUB\nLEVEL",
    0x02: "DIGITAL\nLEVEL",
    0x03: "AUX/HEC 1\nLEVEL",
    0x04: "AUX/HEC 2\nLEVEL",
    0x05: "REAR ATTENUATION\nLEVEL",
}

_HEX_BYTE = re.compile(r"^[0-9A-Fa-f]{2}$")


def assign_label(assign: int) -> str:
    return _ASSIGN_LABELS.get(assign, "")


def decode_frame(data: bytes) -> dict:
    if data == b"\x42\x06":
        return {"name": "volume_wake"}

    if len(data) >= 8 and data[0:4] == b"\xF9\x01\x2B\x04":
        slot = data[4]
        index = data[5]
        code = data[7]
        expect = (VOL_MIN + index + slot) & 0xFF
        return {
            "name": "volume_write",
            "slot": slot,
            "index": index,
            "flag": data[6],
            "code": code,
            "code_matches": code == expect,
        }

    if len(data) < 3 or data[0] != 0x42:
        return {"name": "raw", "len": len(data)}

    kind = data[2]
    b4 = data[4] if len(data) > 4 else None
    b5 = data[5] if len(data) > 5 else None

    if kind == 0xFC and b4 == 0x2A and b5 is not None:
        return {"name": "hs_poll", "sub": b5}
    if kind == 0xFA and b4 == 0x33 and len(data) >= 9:
        return {
            "name": "live_color",
            "slot_id": data[5],
            "r": data[6],
            "g": data[7],
            "b": data[8],
            "mapped_by_firmware": data[5] == 0x90,
        }
    if kind == 0xFA and b4 == 0x2B:
        return {"name": "hs7"}
    if kind == 0xFB and b4 == 0x2A:
        return {"name": "ack_hs0"}
    if kind == 0xFB and b4 == 0x2B:
        return {"name": "ack_hs7"}
    if kind == 0xAF:
        decoded = {"name": "volume_blob"}
        slots = _decode_slots(data)
        if slots is not None:
            decoded["slots"] = slots
        return decoded
    if kind == 0xF9 and b4 == 0x2A and b5 == 0x04:
        indexes = list(data[6:10]) if len(data) >= 10 else list(data[6:])
        return {"name": "volume_snapshot", "indexes": indexes}
    if kind == 0xF9 and b5 == 0x07:
        return {"name": "slot2"}
    if kind == 0xF2:
        return {"name": "f2"}
    if kind == 0xF5 and b5 == 0x09:
        return {"name": "tone"}
    if kind == 0xFC:
        return {"name": "fc"}
    if kind == 0xFD and b4 == 0x33 and b5 is not None:
        return {"name": "fd", "reason": b5, "resync": b5 == 0x02}
    return {"name": f"type_{kind:02X}", "type": kind}


def _decode_slots(data: bytes) -> list[dict] | None:
    # Last field read is the little-endian range at offset+6 of the last slot.
    need = SLOT_BASE0 + (NUM_SLOTS - 1) * SLOT_STRIDE + 8
    if len(data) < need:
        return None
    slots = []
    for slot in range(NUM_SLOTS):
        offset = SLOT_BASE0 + slot * SLOT_STRIDE
        assign = data[offset]
        steps = data[offset + 4]
        if assign == 0xFF or steps == 0:
            slots.append({"slot": slot, "valid": False, "assign": assign})
            continue
        range_db = (data[offset + 6] | (data[offset + 7] << 8)) / 10.0
        slots.append(
            {
                "slot": slot,
                "valid": True,
                "assign": assign,
                "label": assign_label(assign),
                "r": data[offset + 1],
                "g": data[offset + 2],
                "b": data[offset + 3],
                "steps": steps,
                "range_db": range_db,
                "step_db": range_db / steps,
            }
        )
    return slots


def parse_tap_line(line: str) -> dict | None:
    text = line.strip()
    if not text:
        return None
    if text.startswith("#"):
        return {"kind": "banner", "line": text}
    if text.startswith("DROP "):
        count = text[5:].strip()
        if count.isdigit():
            return {"kind": "drop", "count": int(count)}
        return None

    parts = text.split()
    if len(parts) < 2 or not parts[0].isdigit() or parts[1] not in ("K2D", "D2K"):
        return None

    raw: list[int] = []
    trunc = False
    for token in parts[2:]:
        if token == "TRUNC":
            trunc = True
            continue
        if not _HEX_BYTE.match(token):
            return None
        raw.append(int(token, 16))

    data = bytes(raw)
    return {
        "kind": "frame",
        "t_us": int(parts[0]),
        "dir": parts[1],
        "hex": " ".join(f"{b:02X}" for b in data),
        "trunc": trunc,
        "decoded": decode_frame(data),
    }
