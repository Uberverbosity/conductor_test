"""Run the bench host.

The knob firmware in the repository root is left unchanged. This process
listens on localhost, reads the sniffer's USB log, and, when a port is
given, the knob's existing serial text.
"""

from __future__ import annotations

import argparse
import queue
import socket
import threading

from knob_bench.display import DisplayParser
from knob_bench.serial_port import open_quiet
from knob_bench.server import serve
from knob_bench.state import BenchState


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Knob bench host")
    parser.add_argument("--listen", default="127.0.0.1:8734", help="localhost API bind host:port")
    parser.add_argument("--sniffer-port", default=None, help="USB serial device for the listen-only sniffer")
    parser.add_argument("--knob-serial", default=None, help="USB serial device for the knob's existing text log")
    parser.add_argument("--knob-host", default=None, help="Optional future knob TCP address (port 2340/2341)")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args(argv)

    host, port = _split_listen(args.listen)
    state = BenchState()
    stop = threading.Event()

    if args.sniffer_port:
        threading.Thread(
            target=_serial_loop,
            args=(state, args.sniffer_port, args.baud, stop, "sniffer"),
            daemon=True,
        ).start()
    if args.knob_serial:
        threading.Thread(
            target=_serial_loop,
            args=(state, args.knob_serial, args.baud, stop, "knob"),
            daemon=True,
        ).start()
    if args.knob_host:
        threading.Thread(target=_knob_tcp_loop, args=(state, args.knob_host, stop), daemon=True).start()
        threading.Thread(target=_display_loop, args=(state, args.knob_host, stop), daemon=True).start()

    httpd = serve(state, host, port)
    print(f"knob bench at http://{host}:{port}/", flush=True)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        stop.set()
        httpd.shutdown()


def _split_listen(value: str) -> tuple[str, int]:
    host, _, raw_port = value.rpartition(":")
    if not host or not raw_port.isdigit():
        raise SystemExit("--listen must be host:port")
    return host, int(raw_port)


def _serial_loop(state: BenchState, port: str, baud: int, stop: threading.Event, kind: str) -> None:
    set_link = state.set_sniffer_link if kind == "sniffer" else state.set_knob_link
    ingest = state.ingest_tap_line if kind == "sniffer" else state.ingest_knob_line
    while not stop.is_set():
        handle = None
        try:
            handle = open_quiet(port, baud)
            set_link(True, port)
            pending = b""
            while not stop.is_set():
                chunk = handle.read(1024)
                if not chunk:
                    continue
                pending += chunk
                while b"\n" in pending:
                    line, pending = pending.split(b"\n", 1)
                    ingest(line.decode("utf-8", "replace"))
        except Exception as exc:
            set_link(False, port, str(exc))
            if stop.wait(1.0):
                return
        finally:
            if handle is not None:
                handle.close()


def _knob_tcp_loop(state: BenchState, host: str, stop: threading.Event) -> None:
    address = f"{host}:2340"
    while not stop.is_set():
        sock = None
        try:
            sock = socket.create_connection((host, 2340), timeout=2)
            sock.settimeout(0.5)
            replies: queue.Queue[str] = queue.Queue()

            def send(line: str) -> str:
                sock.sendall(line.encode())
                try:
                    return replies.get(timeout=1.0)
                except queue.Empty:
                    return "ERR timeout"

            state.send_command = send
            state.set_knob_link(True, address)
            pending = b""
            while not stop.is_set():
                try:
                    chunk = sock.recv(1024)
                except socket.timeout:
                    continue
                if not chunk:
                    break
                pending += chunk
                while b"\n" in pending:
                    line, pending = pending.split(b"\n", 1)
                    text = line.decode("utf-8", "replace").strip()
                    if text.startswith("OK") or text.startswith("ERR"):
                        replies.put(text)
                    elif text:
                        state.ingest_knob_line(text)
        except OSError as exc:
            state.send_command = None
            state.set_knob_link(False, address, str(exc))
            if stop.wait(2.0):
                return
        finally:
            state.send_command = None
            if sock is not None:
                sock.close()


def _display_loop(state: BenchState, host: str, stop: threading.Event) -> None:
    while not stop.is_set():
        parser = DisplayParser()
        sock = None
        try:
            sock = socket.create_connection((host, 2341), timeout=2)
            sock.settimeout(0.5)
            while not stop.is_set():
                try:
                    chunk = sock.recv(4096)
                except socket.timeout:
                    continue
                if not chunk:
                    break
                state.ingest_frames(parser.feed(chunk))
        except (OSError, ValueError):
            if stop.wait(2.0):
                return
        finally:
            if sock is not None:
                sock.close()


if __name__ == "__main__":
    main()
