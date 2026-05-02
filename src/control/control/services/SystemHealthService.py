#!/usr/bin/env python3
"""SystemHealthService – collects Raspberry Pi system health metrics.

Metrics collected
-----------------
- CPU utilisation (%)
- RAM utilisation (%)
- CPU temperature (°C)      – via psutil sensors or /sys/class/thermal fallback
- Disk utilisation (%)      – root partition
- WiFi latency (ms)         – ping RTT to the local default gateway (router),
                              works on any network without internet access
- WiFi bandwidth (Mbps)     – small HTTP download probe
- Battery % / plugged       – via psutil (useful when running on a laptop or UPS)
- Gateway IP                – the default gateway address that was pinged

Implements :class:`utils.interface.IHealthCheckable` so the HealthMonitorNode
can call ``are_you_ok()`` alongside any other registered services.
"""

import os
import socket
import subprocess
from typing import Dict, List, Optional, Tuple

try:
    import psutil
    _PSUTIL = True
except ImportError:
    _PSUTIL = False

from utils.interface.IHealthCheckable import IHealthCheckable
from utils.Logger import RoverLogger


class SystemHealthService(IHealthCheckable):
    """Reads and exposes Raspberry Pi (Linux) system health metrics.

    Thresholds that mark the system as unhealthy:

    ================  =====
    Metric            Limit
    ================  =====
    CPU temperature   80 °C
    CPU usage         95 %
    RAM usage         95 %
    Disk usage        95 %
    ================  =====
    """

    _PING_COUNT = int(os.getenv('ROVER_PING_COUNT', '3'))
    _BANDWIDTH_URL = os.getenv('ROVER_BANDWIDTH_URL', 'http://speedtest.tele2.net/1MB.zip')

    # Warn thresholds
    _WARN_CPU_TEMP = float(os.getenv('ROVER_WARN_CPU_TEMP', '75.0'))
    _CRIT_CPU_TEMP = float(os.getenv('ROVER_CRIT_CPU_TEMP', '80.0'))
    _CRIT_CPU_PCT = float(os.getenv('ROVER_CRIT_CPU_PCT', '95.0'))
    _CRIT_RAM_PCT = float(os.getenv('ROVER_CRIT_RAM_PCT', '95.0'))
    _CRIT_DISK_PCT = float(os.getenv('ROVER_CRIT_DISK_PCT', '95.0'))

    def __init__(self) -> None:
        self._log = RoverLogger(file_logging=True)
        if not _PSUTIL:
            self._log.warn(
                'psutil not installed – some metrics will return -1. '
                'Run: pip install psutil'
            )
        self._log.info('SystemHealthService ready')

    # ------------------------------------------------------------------
    # IHealthCheckable
    # ------------------------------------------------------------------

    def are_you_ok(self) -> Tuple[bool, Optional[str]]:
        """Return (True, None) if all critical thresholds are within range."""
        issues: List[str] = []
        try:
            m = self.get_metrics()
            if m['cpu_temp_celsius'] >= self._CRIT_CPU_TEMP:
                issues.append(f"CPU temp critical: {m['cpu_temp_celsius']:.1f} °C")
            if m['cpu_percent'] >= self._CRIT_CPU_PCT:
                issues.append(f"CPU overloaded: {m['cpu_percent']:.1f} %")
            if m['ram_percent'] >= self._CRIT_RAM_PCT:
                issues.append(f"RAM critical: {m['ram_percent']:.1f} %")
            if m['disk_percent'] >= self._CRIT_DISK_PCT:
                issues.append(f"Disk nearly full: {m['disk_percent']:.1f} %")
        except Exception as exc:
            return False, f'Health check exception: {exc}'

        if issues:
            return False, '; '.join(issues)
        return True, None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def get_metrics(self) -> Dict[str, float | bool | str]:
        """Return a snapshot of all system metrics as a plain dict."""
        return {
            'cpu_percent': self._cpu_percent(),
            'ram_percent': self._ram_percent(),
            'cpu_temp_celsius': self._cpu_temp(),
            'disk_percent': self._disk_percent(),
            'wifi_latency_ms': self._wifi_latency(),
            'wifi_bandwidth_mbps': self._wifi_bandwidth(),
            'battery_percent': self._battery_percent(),
            'battery_plugged': self._battery_plugged(),
            'gateway_ip': self._get_gateway() or 'unavailable',
        }

    def poll_services(self, services: List[IHealthCheckable]) -> List[str]:
        """Run ``are_you_ok()`` on every registered service.

        Parameters
        ----------
        services:
            List of :class:`IHealthCheckable` instances to interrogate.

        Returns
        -------
        list of str
            Each entry is one of:
            - ``"ClassName:ok"``
            - ``"ClassName:error:<reason>"``
            - ``"ClassName:exception:<exc>"``
        """
        results: List[str] = []
        for svc in services:
            name = type(svc).__name__
            try:
                ok, reason = svc.are_you_ok()
                if ok:
                    results.append(f'{name}:ok')
                else:
                    results.append(f'{name}:error:{reason or "unknown"}')
            except Exception as exc:
                results.append(f'{name}:exception:{exc}')
        return results

    # ------------------------------------------------------------------
    # Private metric readers
    # ------------------------------------------------------------------

    def _cpu_percent(self) -> float:
        if not _PSUTIL:
            return -1.0
        try:
            return float(psutil.cpu_percent(interval=0.1))
        except Exception:
            return -1.0

    def _ram_percent(self) -> float:
        if not _PSUTIL:
            return -1.0
        try:
            return float(psutil.virtual_memory().percent)
        except Exception:
            return -1.0

    def _cpu_temp(self) -> float:
        # psutil – works on Linux/RPi when lm-sensors is present
        if _PSUTIL:
            try:
                temps = psutil.sensors_temperatures()
                for key in ('cpu_thermal', 'coretemp', 'k10temp', 'acpitz'):
                    if key in temps and temps[key]:
                        return float(temps[key][0].current)
            except Exception:
                pass
        # Fallback: RPi thermal zone sysfs
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as fh:
                return float(fh.read().strip()) / 1000.0
        except OSError:
            return -1.0

    def _disk_percent(self) -> float:
        if not _PSUTIL:
            return -1.0
        try:
            return float(psutil.disk_usage('/').percent)
        except Exception:
            return -1.0

    def _wifi_latency(self) -> float:
        """Average RTT to the default gateway (router) in ms, or -1 on failure.

        Pings the local default gateway so latency is measured over the
        physical WiFi/Ethernet link regardless of internet availability.
        """
        target = self._get_gateway()
        if target is None:
            return -1.0
        try:
            result = subprocess.run(
                ['ping', '-c', str(self._PING_COUNT), '-W', '2', target],
                capture_output=True,
                text=True,
                timeout=15,
            )
            for line in result.stdout.splitlines():
                # "rtt min/avg/max/mdev = 10.1/12.3/14.5/1.2 ms"
                if 'avg' in line or 'rtt' in line:
                    segments = line.split('=')[-1].strip().split('/')
                    return float(segments[1])
        except Exception:
            pass
        return -1.0

    @staticmethod
    def _get_gateway() -> Optional[str]:
        """Return the IPv4 default gateway address, or None if undetectable.

        Strategy (in order):
        1. Parse ``/proc/net/route`` (Linux, no extra deps).
        2. Parse ``ip route`` command output.
        3. Fall back to the first hop of a UDP traceroute trick via socket.
        """
        # --- Strategy 1: /proc/net/route (most reliable on Linux/RPi) ---
        try:
            with open('/proc/net/route', 'r') as fh:
                for line in fh.readlines()[1:]:   # skip header
                    parts = line.strip().split()
                    if len(parts) >= 3 and parts[1] == '00000000':  # destination 0.0.0.0
                        # Gateway field is little-endian hex
                        gw_hex = parts[2]
                        gw_bytes = bytes.fromhex(gw_hex)
                        gw_ip = socket.inet_ntoa(gw_bytes[::-1])   # reverse byte order
                        if gw_ip and gw_ip != '0.0.0.0':
                            return gw_ip
        except OSError:
            pass

        # --- Strategy 2: ip route ---
        try:
            out = subprocess.check_output(
                ['ip', 'route', 'show', 'default'],
                text=True, timeout=5
            )
            for token in out.split():
                # line looks like: "default via 192.168.1.1 dev wlan0"
                pass
            parts = out.split()
            via_idx = parts.index('via') if 'via' in parts else -1
            if via_idx != -1 and via_idx + 1 < len(parts):
                return parts[via_idx + 1]
        except Exception:
            pass

        return None

    def _wifi_bandwidth(self) -> float:
        """Download-based bandwidth probe in Mbps, or -1 on failure.

        Downloads a 1 MB test file over HTTP.  Skipped on timeout > 8 s.
        """
        try:
            import time
            import urllib.request

            start = time.monotonic()
            with urllib.request.urlopen(self._BANDWIDTH_URL, timeout=8) as resp:
                data = resp.read()
            elapsed = time.monotonic() - start
            if elapsed > 0:
                return round((len(data) * 8) / (elapsed * 1_000_000), 2)
        except Exception:
            pass
        return -1.0

    def _battery_percent(self) -> float:
        if not _PSUTIL:
            return -1.0
        try:
            bat = psutil.sensors_battery()
            return float(bat.percent) if bat else -1.0
        except Exception:
            return -1.0

    def _battery_plugged(self) -> bool:
        if not _PSUTIL:
            return False
        try:
            bat = psutil.sensors_battery()
            return bool(bat.power_plugged) if bat else False
        except Exception:
            return False
