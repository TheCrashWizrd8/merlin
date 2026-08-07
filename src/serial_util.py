"""
serial_util.py
--------------
Open pyserial ports for USB CDC (ESP32 USB) or Pi GPIO UART (/dev/serial0).
"""

from __future__ import annotations


def is_usb_serial_port(port: str) -> bool:
    p = port.lower()
    return any(x in p for x in ("ttyacm", "ttyusb", "cu.usb", "cu.wchusb"))


def is_gpio_uart_port(port: str) -> bool:
    p = p.lower()
    return any(x in p for x in ("serial0", "ttyama", "ttys0"))


def open_serial_port(
    port: str,
    baud_rate: int = 115200,
    *,
    timeout: float = 0.05,
    write_timeout: float = 0.1,
):
    """
    Open a serial device for ESP telemetry / control.

    GPIO UART (Pi pins 14 TX / 15 RX → /dev/serial0): 8N1, no DTR/RTS.
    USB (/dev/ttyACM0): DTR/RTS held low at open so ESP32-S3 is not reset.
    """
    import serial

    usb = is_usb_serial_port(port)
    kwargs = dict(
        port=port,
        baudrate=baud_rate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=timeout,
        write_timeout=write_timeout,
        rtscts=False,
        dsrdtr=False,
        xonxoff=False,
    )
    if usb:
        kwargs["rts"] = False
        kwargs["dtr"] = False
    ser = serial.Serial(**kwargs)

    if usb:
        try:
            ser.setDTR(False)
            ser.setRTS(False)
        except Exception:
            pass
    else:
        try:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
        except Exception:
            pass

    return ser
