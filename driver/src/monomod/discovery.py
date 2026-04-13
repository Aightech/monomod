"""
Device discovery for MONOMOD.

Primary: mDNS (_monomod._udp.local.) via zeroconf.
Fallback: none yet (firmware doesn't respond to UDP broadcast; the old
MONOMOD.ino "MONOMOD" beacon protocol was never implemented in ESP-IDF).
"""

import socket
import time


def discover_devices(timeout_s: float = 3.0, verbose: bool = True) -> list[dict]:
    """Discover MONOMOD devices on the local network via mDNS.

    Returns list of dicts: [{ip, hostname, port, adc, channels, version}, ...]
    """
    if verbose:
        print(f"[discover] scanning for _monomod._udp for {timeout_s}s...")

    try:
        from zeroconf import Zeroconf, ServiceBrowser
    except ImportError:
        if verbose:
            print("[discover] zeroconf not installed — install with: pip install zeroconf")
        return []

    devices = []
    found = set()

    class Listener:
        def add_service(self, zc, type_, name):
            info = zc.get_service_info(type_, name, timeout=2000)
            if not info:
                if verbose:
                    print(f"[discover]   {name}: got announcement but no info")
                return
            addrs = info.parsed_addresses() if hasattr(info, "parsed_addresses") else []
            if not addrs:
                return
            ip = addrs[0]
            if ip in found:
                return
            found.add(ip)
            props = {}
            if info.properties:
                for k, v in info.properties.items():
                    key = k.decode() if isinstance(k, bytes) else k
                    val = v.decode() if isinstance(v, bytes) else v
                    props[key] = val
            dev = {
                "ip": ip,
                "hostname": (info.server or "").rstrip("."),
                "port": info.port or 5000,
                "adc": props.get("adc", "unknown"),
                "channels": int(props.get("ch", "3")),
                "version": props.get("version", ""),
            }
            devices.append(dev)
            if verbose:
                print(f"[discover]   found: {dev}")

        def remove_service(self, zc, type_, name):
            pass

        def update_service(self, zc, type_, name):
            pass

    zc = Zeroconf()
    try:
        browser = ServiceBrowser(zc, "_monomod._udp.local.", Listener())
        time.sleep(timeout_s)
    finally:
        zc.close()

    if verbose:
        print(f"[discover] scan done — {len(devices)} device(s) found")
    return devices


def resolve_hostname(hostname: str, timeout_s: float = 2.0) -> str | None:
    """Resolve 'monomod-XXYY.local' to an IP via mDNS.

    Returns the IP address string or None on failure.
    """
    try:
        from zeroconf import Zeroconf
    except ImportError:
        return None

    zc = Zeroconf()
    try:
        info = zc.get_service_info("_monomod._udp.local.",
                                   f"{hostname}._monomod._udp.local.",
                                   timeout=int(timeout_s * 1000))
        if info:
            addrs = info.parsed_addresses() if hasattr(info, "parsed_addresses") else []
            return addrs[0] if addrs else None
    finally:
        zc.close()
    return None
