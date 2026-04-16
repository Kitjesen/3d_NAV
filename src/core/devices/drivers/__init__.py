"""Built-in device drivers. Importing this registers all driver classes."""

from core.devices.drivers import serial_nmea0183

__all__ = ["serial_nmea0183"]
