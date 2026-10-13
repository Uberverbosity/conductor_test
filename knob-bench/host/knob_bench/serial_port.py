"""Open a USB serial port without pulsing DTR/RTS.

ESP32 native USB treats DTR as a reset into the ROM bootloader. The sniffer
and the knob both need the line left alone, matching monitor_dtr = 0.
"""

from __future__ import annotations

import serial


def open_quiet(port: str, baud: int, factory=serial.Serial):
    handle = factory()
    handle.port = port
    handle.baudrate = baud
    handle.timeout = 0.5
    handle.dsrdtr = False
    handle.rtscts = False
    handle.dtr = False
    handle.rts = False
    handle.open()
    handle.dtr = False
    handle.rts = False
    return handle
