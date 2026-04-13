"""MONOMOD — Python driver for standalone wireless EMG modules."""

from .device import Device
from .discovery import discover_devices

__version__ = "0.1.0"
__all__ = ["Device", "discover_devices"]
