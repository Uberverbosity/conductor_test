from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SOURCE = (ROOT / "sniffer" / "src" / "main.cpp").read_text()
INI = (ROOT / "sniffer" / "platformio.ini").read_text()


def test_firmware_cannot_drive_the_uart():
    assert "uart_write" not in SOURCE
    assert "UART_PIN_NO_CHANGE" in SOURCE
    assert "timeout_flag" in SOURCE


def test_usb_pins_are_not_the_listen_pins():
    assert "ARDUINO_USB_MODE=0" in INI
    assert "ARDUINO_USB_CDC_ON_BOOT=1" in INI
    assert 'USB_PRODUCT=\'"sniffer"\'' in INI
    assert "TAP_RX_K2D=4" in INI
    assert "TAP_RX_D2K=5" in INI
    assert "TAP_RX_TOUT_BITS=70" in INI
