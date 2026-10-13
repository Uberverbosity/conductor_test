"""Localhost API and a single page that shows the bench."""

from __future__ import annotations

import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

from knob_bench.commands import encode_command
from knob_bench.display import DisplayParser
from knob_bench.state import BenchState

_PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>Knob bench</title>
<style>
  body { margin: 24px; font: 14px/1.4 ui-sans-serif, system-ui, sans-serif; background: #111; color: #eee; }
  h1 { font-size: 20px; font-weight: 590; margin: 0 0 8px; }
  p { margin: 0 0 16px; color: #aaa; max-width: 62em; }
  .layout { display: flex; gap: 24px; align-items: flex-start; flex-wrap: wrap; }
  img { width: 240px; height: 240px; background: #000; border-radius: 50%; }
  pre { margin: 0; white-space: pre-wrap; font: 12px/1.45 ui-monospace, monospace; }
  section { min-width: 280px; flex: 1; }
  h2 { font-size: 14px; margin: 0 0 8px; }
  button, input { font: inherit; background: #222; color: #eee; border: 1px solid #444; padding: 4px 8px; }
  .row { display: flex; gap: 8px; margin: 8px 0 16px; }
  .note { color: #888; font-size: 12px; }
</style>
</head>
<body>
  <h1>Knob bench</h1>
  <p id="note">The round picture is reconstructed from the knob log until a pixel mirror is connected. Packets come from the listen-only sniffer.</p>
  <div class="layout">
    <div>
      <img id="screen" alt="Knob display" width="240" height="240">
      <div class="row">
        <button id="left" type="button">Turn −1</button>
        <button id="right" type="button">Turn +1</button>
        <button id="click" type="button">Click</button>
      </div>
      <div class="note" id="cmd"></div>
    </div>
    <section>
      <h2>Status</h2>
      <pre id="status">…</pre>
    </section>
    <section>
      <h2>Packets</h2>
      <pre id="packets"></pre>
    </section>
    <section>
      <h2>Knob log</h2>
      <pre id="log"></pre>
    </section>
  </div>
<script>
const statusEl = document.getElementById("status");
const packetsEl = document.getElementById("packets");
const logEl = document.getElementById("log");
const noteEl = document.getElementById("note");
const cmdEl = document.getElementById("cmd");
let packetSeq = 0;
let logSeq = 0;
const packetLines = [];
const logLines = [];

function renderList(el, lines) {
  el.textContent = lines.slice(-40).join("\\n");
}

async function tick() {
  const status = await fetch("/status").then((r) => r.json());
  statusEl.textContent = JSON.stringify(status, null, 2);
  if (status.displaySource === "mirror") {
    noteEl.textContent = "The round picture is the knob's LVGL flush.";
  }
  document.getElementById("screen").src = "/display.png?t=" + Date.now();
  const packets = await fetch("/packets?since=" + packetSeq).then((r) => r.json());
  for (const pkt of packets.packets) {
    packetSeq = pkt.seq;
    packetLines.push(pkt.t_us + " " + pkt.dir + " " + (pkt.name || "") + " " + pkt.hex);
  }
  renderList(packetsEl, packetLines);
  const logs = await fetch("/log?since=" + logSeq).then((r) => r.json());
  for (const row of logs.lines) {
    logSeq = row.seq;
    logLines.push(row.line);
  }
  renderList(logEl, logLines);
}

async function send(body) {
  const res = await fetch("/input", {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(body)});
  const text = await res.text();
  cmdEl.textContent = res.status + " " + text;
}

document.getElementById("left").onclick = () => send({turn: -1});
document.getElementById("right").onclick = () => send({turn: 1});
document.getElementById("click").onclick = () => send({click: true});
tick();
setInterval(tick, 500);
</script>
</body>
</html>
"""


def make_handler(state: BenchState, display_parser: DisplayParser):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, fmt: str, *args) -> None:
            return

        def do_GET(self) -> None:
            parsed = urlparse(self.path)
            query = parse_qs(parsed.query)
            if parsed.path == "/":
                body = _PAGE.encode()
                self._send(200, "text/html; charset=utf-8", body)
                return
            if parsed.path == "/status":
                self._send(200, "application/json", _json(state.status()))
                return
            if parsed.path == "/display.png":
                self._send(200, "image/png", state.png(), cache=False)
                return
            if parsed.path == "/display/events":
                since = _since(query)
                self._send(200, "application/json", _json({"events": state.events_since(since)}))
                return
            if parsed.path == "/packets":
                since = _since(query)
                self._send(200, "application/json", _json({"packets": state.packets_since(since)}))
                return
            if parsed.path == "/log":
                since = _since(query)
                self._send(200, "application/json", _json({"lines": state.logs_since(since)}))
                return
            self._send(404, "application/json", _json({"error": "not-found"}))

        def do_POST(self) -> None:
            parsed = urlparse(self.path)
            raw = self._read_body()
            if parsed.path == "/capture":
                self._send(200, "image/gif", state.gif(), cache=False)
                return
            if parsed.path == "/input":
                try:
                    body = json.loads(raw.decode() or "{}")
                    line = encode_command(body)
                except (ValueError, json.JSONDecodeError):
                    self._send(400, "application/json", _json({"ok": False, "error": "ERR range"}))
                    return
                send = state.send_command
                if send is None:
                    self._send(503, "application/json", _json({"ok": False, "error": "knob-down", "line": line.strip()}))
                    return
                reply = send(line)
                self._send(200, "application/json", _json({"ok": True, "line": line.strip(), "reply": reply.strip()}))
                return
            if parsed.path == "/display":
                try:
                    frames = display_parser.feed(raw)
                except ValueError as exc:
                    self._send(400, "application/json", _json({"ok": False, "error": str(exc)}))
                    return
                state.ingest_frames(frames)
                self._send(200, "application/json", _json({"ok": True, "frames": len(frames)}))
                return
            self._send(404, "application/json", _json({"error": "not-found"}))

        def _read_body(self) -> bytes:
            length = int(self.headers.get("Content-Length", "0") or 0)
            if length < 0 or length > 2_000_000:
                return b""
            return self.rfile.read(length) if length else b""

        def _send(self, code: int, content_type: str, body: bytes, cache: bool = True) -> None:
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            if not cache:
                self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

    return Handler


def _json(payload: dict) -> bytes:
    return json.dumps(payload).encode()


def _since(query: dict) -> int:
    raw = query.get("since", ["0"])[0]
    try:
        return int(raw)
    except ValueError:
        return 0


class BenchServer(ThreadingHTTPServer):
    allow_reuse_address = True


def serve(state: BenchState, host: str = "127.0.0.1", port: int = 8734) -> BenchServer:
    parser = DisplayParser()
    httpd = BenchServer((host, port), make_handler(state, parser))
    return httpd
