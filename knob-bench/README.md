# Knob bench

A separate project from the knob firmware at the repository root. That firmware is left as it is.

This tree has two pieces:

- `sniffer/` — ESP32-S3 firmware that only listens. Two UART RX pins tap the direct knob-to-DSP pair. Transmit pins are not connected, and the firmware never writes those UARTs.
- `host/` — a program on the PC. It serves `http://127.0.0.1:8734/`, reads the sniffer's USB log, and can read the knob's existing serial text.

## Wiring

The knob TX/RX/GND cable to the DSP stays a direct connection. The sniffer sits in parallel:

| Conductor | Sniffer |
|---|---|
| Knob TX / DSP RX | RX, logged as `K2D`. Default GPIO 4. |
| DSP TX / knob RX | RX, logged as `D2K`. Default GPIO 5. |
| Ground | Sniffer ground |
| Sniffer TX | Open |

Change `TAP_RX_K2D` and `TAP_RX_D2K` in `sniffer/platformio.ini` to the pins on the tap board before flashing. GPIO 19 and 20 are native USB on the S3, so they cannot be listen pins.

One USB cable from the sniffer to the PC powers it and carries the packet log. The knob stays on the WiFi link already used for code pushes. Its USB receptacle stays unplugged while DSP RX is GPIO 20, because that pin is USB D+ on an S3.

Both UARTs are 3.3 V. The idle threshold is 70 bit times at 230400, about 304 µs. That sits just past the knob's 300 µs frame gap, and it still splits the 600 µs pause in a volume write.

## Build the sniffer

```bash
pio run -d knob-bench/sniffer
pio run -d knob-bench/sniffer -t upload --upload-port /dev/ttyACM0
```

Opening the port must not pulse DTR. `monitor_dtr` and `monitor_rts` are 0, and the firmware disables the USB reboot-on-DTR. The USB product string is `sniffer`.

Each frame is one line: `<time_us> K2D <hex>` or `D2K`. `DROP n` is the count of frames the logger could not keep. `TRUNC` marks a frame longer than 512 bytes.

## Run the host

```bash
python3 -m pip install -e "knob-bench/host[dev]"
python3 -m knob_bench --sniffer-port /dev/ttyACM0 --knob-serial /dev/ttyACM1
```

Run that from the repository root after install, or from `knob-bench/host` with `python3 -m knob_bench`.

`--knob-serial` reads the text the current knob firmware already prints (`[UI]`, `[HELIX]`, `[ENC]`, `[RX]`, `[TX]`). The page draws a dial from those lines. That drawing is a reconstruction, not a copy of the panel. `displayFresh` stays false for it.

`--knob-host` is for a later knob build that listens on TCP 2340 (log and `TURN` / `CLICK` / `TOUCH` lines) and streams LVGL rectangles on TCP 2341. The stable firmware does not do that, so leave this unset. `POST /input` then returns `knob-down`.

## Tests

```bash
python3 -m pytest knob-bench/host/tests
```
