"""Built-in protocol decoders. Importing this module registers all decoders
so they show up in decoder_registry()."""

from core.devices.decoders import nmea  # noqa: F401 — registers nmea0183

__all__ = ["nmea"]
