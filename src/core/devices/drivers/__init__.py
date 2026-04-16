"""Built-in device drivers. Importing this registers all driver classes."""

from core.devices.drivers import serial_nmea0183  # noqa: F401

__all__ = ["serial_nmea0183"]
