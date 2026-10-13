"""Shared bench state. HTTP handlers and serial readers both call in here."""

from __future__ import annotations

import threading
import time

from knob_bench.decode import parse_tap_line
from knob_bench.display import Canvas
from knob_bench.knob_log import parse_knob_line


class BenchState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.canvas = Canvas()
        self.sniffer_connected = False
        self.sniffer_port: str | None = None
        self.sniffer_error: str | None = None
        self.sniffer_drops = 0
        self.sniffer_banner: str | None = None
        self.knob_connected = False
        self.knob_port: str | None = None
        self.knob_error: str | None = None
        self.helix_ready: bool | None = None
        self.active_slot: int | None = None
        self.dial: int | None = None
        self.last_encoder_delta: int | None = None
        self.button_presses = 0
        self.volumes: dict[int, dict] = {}
        self._packets: list[dict] = []
        self._logs: list[dict] = []
        self._packet_seq = 0
        self._log_seq = 0
        self.send_command = None

    def set_sniffer_link(self, connected: bool, port: str | None, error: str | None = None) -> None:
        with self._lock:
            self.sniffer_connected = connected
            self.sniffer_port = port
            self.sniffer_error = error

    def set_knob_link(self, connected: bool, port: str | None, error: str | None = None) -> None:
        with self._lock:
            self.knob_connected = connected
            self.knob_port = port
            self.knob_error = error

    def ingest_tap_line(self, line: str) -> None:
        parsed = parse_tap_line(line)
        if parsed is None:
            return
        with self._lock:
            if parsed["kind"] == "banner":
                self.sniffer_banner = parsed["line"]
                return
            if parsed["kind"] == "drop":
                self.sniffer_drops = parsed["count"]
                return
            self._packet_seq += 1
            self._packets.append(
                {
                    "seq": self._packet_seq,
                    "t_us": parsed["t_us"],
                    "dir": parsed["dir"],
                    "hex": parsed["hex"],
                    "trunc": parsed["trunc"],
                    "name": parsed["decoded"].get("name"),
                    "decoded": parsed["decoded"],
                    "host_ms": int(time.time() * 1000),
                }
            )
            if len(self._packets) > 2000:
                self._packets = self._packets[-2000:]

    def ingest_knob_line(self, line: str) -> None:
        parsed = parse_knob_line(line)
        if not parsed["line"]:
            return
        with self._lock:
            self._log_seq += 1
            entry = {"seq": self._log_seq, "line": parsed["line"], "host_ms": int(time.time() * 1000)}
            if "wire" in parsed:
                entry["wire"] = parsed["wire"]
            self._logs.append(entry)
            if len(self._logs) > 2000:
                self._logs = self._logs[-2000:]
            self._apply_knob(parsed)
            self._refresh_reconstruction()

    def _apply_knob(self, parsed: dict) -> None:
        if "dial" in parsed:
            self.dial = max(0, min(100, parsed["dial"]))
        if "active_slot" in parsed:
            self.active_slot = parsed["active_slot"]
        if "ready" in parsed:
            self.helix_ready = parsed["ready"]
        if "encoder_delta" in parsed:
            self.last_encoder_delta = parsed["encoder_delta"]
        if parsed.get("button") == "pressed":
            self.button_presses += 1
        if "slot_color" in parsed:
            color = parsed["slot_color"]
            slot = self.volumes.setdefault(color["slot"], {"slot": color["slot"]})
            slot["r"] = color["r"]
            slot["g"] = color["g"]
            slot["b"] = color["b"]
        if "volume" in parsed:
            vol = parsed["volume"]
            current = self.volumes.setdefault(vol["slot"], {})
            current.update(vol)

    def _refresh_reconstruction(self) -> None:
        color = (136, 136, 136)
        known = False
        label = ""
        if self.active_slot is not None:
            vol = self.volumes.get(self.active_slot, {})
            if "r" in vol:
                color = (vol["r"], vol["g"], vol["b"])
                known = True
            label = vol.get("label") or f"VOL {self.active_slot + 1}"
        self.canvas.set_reconstructed(self.dial, color, label, known)

    def ingest_frames(self, frames: list[dict]) -> None:
        with self._lock:
            for frame in frames:
                self.canvas.apply_rect(
                    frame["x"],
                    frame["y"],
                    frame["w"],
                    frame["h"],
                    frame["pixels"],
                    frame["seq"],
                    frame["t_ms"],
                    frame["flags"],
                )

    def png(self) -> bytes:
        with self._lock:
            return self.canvas.png()

    def gif(self) -> bytes:
        with self._lock:
            return self.canvas.gif()

    def events_since(self, seq: int) -> list[dict]:
        with self._lock:
            return self.canvas.events_since(seq)

    def packets_since(self, seq: int) -> list[dict]:
        with self._lock:
            return [pkt for pkt in self._packets if pkt["seq"] > seq]

    def logs_since(self, seq: int) -> list[dict]:
        with self._lock:
            return [row for row in self._logs if row["seq"] > seq]

    def status(self) -> dict:
        with self._lock:
            return {
                "sniffer": {
                    "connected": self.sniffer_connected,
                    "port": self.sniffer_port,
                    "error": self.sniffer_error,
                    "drops": self.sniffer_drops,
                    "banner": self.sniffer_banner,
                },
                "knob": {
                    "connected": self.knob_connected,
                    "port": self.knob_port,
                    "error": self.knob_error,
                    "commandLink": self.send_command is not None,
                },
                "helixReady": self.helix_ready,
                "activeSlot": self.active_slot,
                "dial": self.dial,
                "lastEncoderDelta": self.last_encoder_delta,
                "buttonPresses": self.button_presses,
                "displaySource": self.canvas.source,
                "displayFresh": self.canvas.fresh,
                "droppedFrames": self.canvas.dropped_frames,
                "colorKnown": self.canvas.color_known,
            }
