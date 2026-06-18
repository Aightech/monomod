"""Cross-device clock-sync offset math (min-RTT)."""
import time

from monomod.device import Device


class _StubCtrl:
    """Returns a noisy RTT; one sample much larger to verify min-RTT selection."""
    def __init__(self):
        self.n = 0

    def ping_clock(self, timeout=1.0):
        self.n += 1
        rtt = 0.050 if self.n == 3 else 0.004
        dev_ts = int((time.time() - 10.0) * 1e6) & 0xFFFFFFFF
        return (rtt, dev_ts)


def test_sync_picks_min_rtt():
    d = Device()
    d._ctrl = _StubCtrl()
    assert d.sync_clock(n=5) is True
    assert d.clock_rtt_s < 0.01          # picked the low-RTT sample
    # device ts ~10 s ago should map back to ~now
    ts = int((time.time() - 10.0) * 1e6) & 0xFFFFFFFF
    assert abs(d.device_to_host(ts) - time.time()) < 0.1


def test_device_to_host_none_when_unsynced():
    assert Device().device_to_host(123456) is None
