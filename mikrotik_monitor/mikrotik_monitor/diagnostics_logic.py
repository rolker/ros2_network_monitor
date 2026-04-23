# Copyright 2024 Roland Arsenault
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""
Pure-Python diagnostic logic for the MikroTik monitor.

Avoids ROS 2 runtime dependencies (rclpy) so it can be unit-tested
against hand-built cache snapshots.  Still imports DiagnosticStatus for
the standard level constants.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


# Task-name suffix builders.  Centralized so the node and tests agree on
# the exact strings used as dynamic-membership keys.

def interface_task_name(name_prefix: str, iface_name: str) -> str:
    """Build the full task name for a per-interface diagnostic."""
    return f'{name_prefix}: interface/{iface_name}'


def wireless_task_name(name_prefix: str, iface: str, mac: str) -> str:
    """Build the full task name for a per-wireless-registration diagnostic."""
    return f'{name_prefix}: wireless/{iface}/{mac}'


# System-resource fields to surface as KeyValue pairs.
_SYSTEM_FIELDS = (
    'board-name', 'version', 'uptime',
    'cpu-load', 'cpu-count', 'cpu-frequency',
    'free-memory', 'total-memory',
    'free-hdd-space', 'total-hdd-space',
)

# Interface fields to surface as KeyValue pairs.
_INTERFACE_FIELDS = (
    'type', 'mtu', 'running', 'disabled',
    'tx-byte', 'rx-byte',
    'tx-packet', 'rx-packet',
    'tx-error', 'rx-error',
    'tx-drop', 'rx-drop',
    'link-downs',
)

# Wireless registration fields to surface as KeyValue pairs.
_WIRELESS_FIELDS = (
    'interface', 'mac-address',
    'signal-strength', 'signal-to-noise',
    'noise-floor',
    'tx-rate', 'rx-rate',
    'tx-ccq', 'rx-ccq',
    'uptime',
    'bytes', 'packets',
    'frames',
)


@dataclass
class CachedStatus:
    """
    Latest parsed device response plus metadata.

    The poll callback replaces this whole-dataclass under a lock so task
    callbacks on the rclpy thread only ever see a consistent snapshot.
    ``poll_monotonic == 0.0`` means "no poll has completed yet."
    """

    system_resource: Optional[dict] = None   # from /system/resource
    system_health: Optional[object] = None   # from /system/health (list or dict)
    interfaces: list[dict] = field(default_factory=list)   # from /interface
    wireless: list[dict] = field(default_factory=list)     # from /interface/wireless/registration-table
    poll_monotonic: float = 0.0
    poll_wall_iso: str = ''
    error_message: Optional[str] = None  # non-None if last poll attempt failed


def parse_snr(value) -> Optional[float]:
    """Parse an SNR string like '22@HT40' or '22 dB' to float."""
    if value is None:
        return None
    try:
        return float(str(value).split('@')[0].split()[0])
    except (ValueError, IndexError):
        return None


def wireless_quality(snr: Optional[float]) -> tuple[int, str]:
    """
    Map SNR to a diagnostic level and a signal-bars string.

    Thresholds tuned for 5 GHz point-to-point links:
      Excellent: SNR > 30
      Good:      SNR >= 20
      Fair:      SNR >= 15
      Poor:      SNR >= 10
      Very poor: below
    """
    bar_chars = ['▁', '▂', '▃', '▅', '█']
    if snr is None:
        return DiagnosticStatus.WARN, '?'
    if snr > 30:
        n = 5
    elif snr >= 20:
        n = 4
    elif snr >= 15:
        n = 3
    elif snr >= 10:
        n = 2
    else:
        n = 1
    bars = ''.join(bar_chars[:n]) + ''.join('·' for _ in range(5 - n))
    if n >= 4:
        level = DiagnosticStatus.OK
    elif n >= 2:
        level = DiagnosticStatus.WARN
    else:
        level = DiagnosticStatus.ERROR
    return level, bars


def _last_query_kv(cache: CachedStatus) -> KeyValue:
    return KeyValue(
        key='last_query_time',
        value=cache.poll_wall_iso or 'never',
    )


def _is_cache_stale(cache: CachedStatus, stale_timeout_sec: float,
                   now_monotonic: float) -> Optional[str]:
    """
    Return a STALE message if the cache is stale, else None.

    ``poll_monotonic == 0.0`` means "no poll completed yet" — report
    STALE with that reason.  Otherwise compare cache age against
    ``stale_timeout_sec``.
    """
    if cache.poll_monotonic == 0.0:
        return 'no successful poll yet'
    age = now_monotonic - cache.poll_monotonic
    if age > stale_timeout_sec:
        return (
            f'cached data {age:.1f}s old '
            f'(stale_timeout_sec={stale_timeout_sec})'
        )
    return None


def synthesize_connection_status(
    cache: CachedStatus,
    stale_timeout_sec: float,
    now_monotonic: float,
) -> tuple[int, str, list[KeyValue]]:
    """
    Render the top-level connection health task.

    Replaces the bug-creating "rename-on-error" status that the pre-Updater
    node emitted.  Name stays fixed as ``<prefix>: connection``; level
    reflects whether the last poll succeeded.
    """
    kvs: list[KeyValue] = [_last_query_kv(cache)]

    stale_msg = _is_cache_stale(cache, stale_timeout_sec, now_monotonic)
    if stale_msg is not None:
        return DiagnosticStatus.STALE, stale_msg, kvs

    if cache.error_message:
        return DiagnosticStatus.ERROR, cache.error_message, kvs

    return DiagnosticStatus.OK, 'reachable', kvs


def synthesize_system_status(
    cache: CachedStatus,
    stale_timeout_sec: float,
    now_monotonic: float,
) -> tuple[int, str, list[KeyValue]]:
    """Render the system/resource task (board, CPU, memory, health)."""
    kvs: list[KeyValue] = [_last_query_kv(cache)]

    stale_msg = _is_cache_stale(cache, stale_timeout_sec, now_monotonic)
    if stale_msg is not None:
        return DiagnosticStatus.STALE, stale_msg, kvs

    resource = cache.system_resource
    if resource is None:
        # Poll succeeded but system_resource wasn't populated — shouldn't
        # happen in normal operation but fail safely.
        return (
            DiagnosticStatus.ERROR,
            'system_resource not cached',
            kvs,
        )

    for key in _SYSTEM_FIELDS:
        if key in resource:
            kvs.append(KeyValue(key=key, value=str(resource[key])))

    # Optional health data
    health = cache.system_health
    if isinstance(health, list):
        for entry in health:
            name = entry.get('name', '')
            value = entry.get('value', '')
            if name and value:
                kvs.append(KeyValue(key=name, value=str(value)))
    elif isinstance(health, dict):
        for key, value in health.items():
            if key != '.id':
                kvs.append(KeyValue(key=key, value=str(value)))

    message = resource.get('board-name', 'unknown')
    return DiagnosticStatus.OK, message, kvs


def synthesize_interface_status(
    cache: CachedStatus,
    iface_name: str,
    stale_timeout_sec: float,
    now_monotonic: float,
) -> tuple[int, str, list[KeyValue]]:
    """Render the per-interface task."""
    kvs: list[KeyValue] = [_last_query_kv(cache)]

    stale_msg = _is_cache_stale(cache, stale_timeout_sec, now_monotonic)
    if stale_msg is not None:
        return DiagnosticStatus.STALE, stale_msg, kvs

    iface = _find_by_key(cache.interfaces, 'name', iface_name)
    if iface is None:
        # Interface disappeared from the last successful poll but its
        # task is still registered (grace period hasn't elapsed).
        return (
            DiagnosticStatus.STALE,
            f'interface {iface_name!r} not in latest poll',
            kvs,
        )

    running = str(iface.get('running', 'false')).lower() == 'true'
    disabled = str(iface.get('disabled', 'false')).lower() == 'true'

    if disabled:
        level, message = DiagnosticStatus.WARN, 'Disabled'
    elif not running:
        level, message = DiagnosticStatus.WARN, 'Not running'
    else:
        level, message = DiagnosticStatus.OK, 'Running'

    for key in _INTERFACE_FIELDS:
        if key in iface:
            kvs.append(KeyValue(key=key, value=str(iface[key])))

    return level, message, kvs


def synthesize_wireless_status(
    cache: CachedStatus,
    iface: str,
    mac: str,
    stale_timeout_sec: float,
    now_monotonic: float,
) -> tuple[int, str, list[KeyValue]]:
    """Render the per-wireless-registration task."""
    kvs: list[KeyValue] = [_last_query_kv(cache)]

    stale_msg = _is_cache_stale(cache, stale_timeout_sec, now_monotonic)
    if stale_msg is not None:
        return DiagnosticStatus.STALE, stale_msg, kvs

    reg = _find_wireless(cache.wireless, iface, mac)
    if reg is None:
        return (
            DiagnosticStatus.STALE,
            f'registration {iface}/{mac} not in latest poll',
            kvs,
        )

    snr = parse_snr(reg.get('signal-to-noise'))
    level, bars = wireless_quality(snr)
    snr_str = f' SNR {snr:.0f}dB' if snr is not None else ''
    message = f'Associated{snr_str} {bars}'

    for key in _WIRELESS_FIELDS:
        if key in reg:
            kvs.append(KeyValue(key=key, value=str(reg[key])))

    return level, message, kvs


def diff_dynamic_membership(
    observed: set[str],
    registered: set[str],
    last_seen: dict[str, float],
    grace_sec: float,
    now_monotonic: float,
) -> tuple[set[str], set[str], dict[str, float]]:
    """
    Compute dynamic-task membership deltas.

    Returns ``(to_add, to_remove, new_last_seen)``:

    - ``to_add`` — names in ``observed`` that are not in ``registered``.
    - ``to_remove`` — names in ``registered`` whose last-observed time
      is older than ``grace_sec``.  A name that reappears before the
      grace period lapses is not removed — this prevents Updater churn
      on transient flaps (e.g., interfaces cycling during mwan3 handoffs).
    - ``new_last_seen`` — updated mapping: every currently-observed
      name maps to ``now_monotonic``; names still within the grace
      period retain their old timestamp; names past the grace period
      are dropped.

    Pure function so tests can drive the membership logic without
    Updater or rclpy.
    """
    to_add = observed - registered
    new_last_seen: dict[str, float] = {}
    to_remove: set[str] = set()

    for name in registered | observed:
        if name in observed:
            new_last_seen[name] = now_monotonic
        else:
            # Not observed this poll — check grace period
            previous = last_seen.get(name, now_monotonic)
            if now_monotonic - previous > grace_sec:
                to_remove.add(name)
            else:
                new_last_seen[name] = previous

    return to_add, to_remove, new_last_seen


def _find_by_key(items: list[dict], key: str, value: str) -> Optional[dict]:
    for item in items:
        if item.get(key) == value:
            return item
    return None


def _find_wireless(items: list[dict], iface: str, mac: str) -> Optional[dict]:
    for item in items:
        if item.get('interface') == iface and item.get('mac-address') == mac:
            return item
    return None
