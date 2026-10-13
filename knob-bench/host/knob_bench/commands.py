"""Lines the host sends when a knob build is listening on TCP 2340.

The stable knob firmware does not speak these lines. The encoder and button
on that build keep working as they do today.
"""

from __future__ import annotations


def encode_command(body: dict) -> str:
    if "turn" in body:
        clicks = body["turn"]
        if isinstance(clicks, bool) or not isinstance(clicks, int):
            raise ValueError("ERR range")
        if clicks < -128 or clicks > 127:
            raise ValueError("ERR range")
        return f"TURN {clicks}\n"
    if body.get("click") is True:
        return "CLICK\n"
    if body.get("down") is True:
        return "DOWN\n"
    if body.get("up") is True:
        return "UP\n"
    touch = body.get("touch")
    if isinstance(touch, dict):
        try:
            x = int(touch["x"])
            y = int(touch["y"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("ERR range") from exc
        phase = touch.get("phase")
        if phase not in ("down", "move", "up") or not (0 <= x <= 239 and 0 <= y <= 239):
            raise ValueError("ERR range")
        return f"TOUCH {x} {y} {phase}\n"
    raise ValueError("ERR range")
