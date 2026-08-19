"""USB GPS vs ESP serial identity and NMEA parse."""

import unittest

from src.gps_reader import _is_reserved_esp_port, parse_nmea_line
from src.serial_util import is_esp_usb_device, is_gps_usb_device
from src.sub_state import SubState

_GPS = "/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_7_-_GPS_GNSS_Receiver-if00"
_ESP = "/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_EC_DA_3B_5A_FC_60-if00"


class SerialIdentityTests(unittest.TestCase):
    def test_ublox_is_gps_not_esp(self):
        self.assertTrue(is_gps_usb_device(_GPS))
        self.assertFalse(is_esp_usb_device(_GPS))

    def test_espressif_is_esp_not_gps(self):
        self.assertTrue(is_esp_usb_device(_ESP))
        self.assertFalse(is_gps_usb_device(_ESP))

    def test_gps_not_skipped_even_if_claimed_as_esp_port(self):
        self.assertFalse(_is_reserved_esp_port(_GPS, esp_port=_GPS))
        self.assertFalse(_is_reserved_esp_port(_GPS, esp_port="/dev/ttyACM0"))

    def test_esp_by_id_is_skipped(self):
        self.assertTrue(_is_reserved_esp_port(_ESP, esp_port=_ESP))


class NmeaParseTests(unittest.TestCase):
    def test_void_gga_records_sats_without_fix(self):
        state = SubState()
        ok = parse_nmea_line(
            "$GPGGA,113139.00,,,,,0,00,99.99,,,,,,*6E",
            state,
        )
        self.assertFalse(ok)
        self.assertFalse(state.gps_connected)
        self.assertEqual(state.gps.satellites, 0)
        self.assertEqual(state.gps.fix_quality, 0)
        self.assertAlmostEqual(state.gps.hdop, 99.99)

    def test_valid_gga_sets_fix(self):
        state = SubState()
        ok = parse_nmea_line(
            "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47",
            state,
        )
        self.assertTrue(ok)
        self.assertTrue(state.gps_connected)
        self.assertAlmostEqual(state.gps.lat, 48.1173, places=4)
        self.assertAlmostEqual(state.gps.lon, 11.516666, places=4)
        self.assertEqual(state.gps.satellites, 8)

    def test_void_rmc_ignored(self):
        state = SubState()
        self.assertFalse(parse_nmea_line("$GPRMC,113139.00,V,,,,,,,190826,,,N*71", state))
        self.assertFalse(state.gps_connected)


if __name__ == "__main__":
    unittest.main()
