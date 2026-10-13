from knob_bench.decode import decode_frame, parse_tap_line

HS0 = bytes.fromhex("4203FC012A002A")
HS7 = bytes.fromhex("4205FA012B0101012E")


def test_handshake_poll_matches_firmware_bytes():
    decoded = decode_frame(HS0)
    assert decoded["name"] == "hs_poll"
    assert decoded["sub"] == 0x00


def test_hs7_is_not_a_color_frame():
    assert decode_frame(HS7)["name"] == "hs7"


def test_volume_wake_and_body_are_separate_frames():
    assert decode_frame(bytes.fromhex("4206"))["name"] == "volume_wake"
    body = bytes.fromhex("F9012B0400050135")
    decoded = decode_frame(body)
    assert decoded["name"] == "volume_write"
    assert decoded["slot"] == 0
    assert decoded["index"] == 5
    assert decoded["code_matches"] is True


def test_live_color_records_slots_the_firmware_ignores():
    frame = bytes.fromhex("4200FA003391010203")
    decoded = decode_frame(frame)
    assert decoded["name"] == "live_color"
    assert decoded["slot_id"] == 0x91
    assert decoded["mapped_by_firmware"] is False


def test_fd_reason_02_is_a_resync():
    frame = bytes.fromhex("4206FD01330200")
    decoded = decode_frame(frame)
    assert decoded == {"name": "fd", "reason": 0x02, "resync": True}


def test_volume_blob_slots():
    data = bytearray(60)
    data[0] = 0x42
    data[2] = 0xAF
    data[22:30] = bytes([0x00, 10, 20, 30, 40, 0x00, 0x20, 0x03])
    data[32] = 0xFF
    slots = decode_frame(bytes(data))["slots"]
    assert slots[0]["valid"] is True
    assert slots[0]["label"] == "MASTER\nVOLUME"
    assert slots[0]["range_db"] == 80.0
    assert slots[0]["step_db"] == 2.0
    assert slots[1]["valid"] is False


def test_tap_line_round_trip():
    parsed = parse_tap_line("1000 K2D 42 03 FC 01 2A 00 2A")
    assert parsed["kind"] == "frame"
    assert parsed["dir"] == "K2D"
    assert parsed["decoded"]["name"] == "hs_poll"
    assert parse_tap_line("DROP 4") == {"kind": "drop", "count": 4}
    assert parse_tap_line("# knob-bench sniffer")["kind"] == "banner"
    assert parse_tap_line("nope") is None
