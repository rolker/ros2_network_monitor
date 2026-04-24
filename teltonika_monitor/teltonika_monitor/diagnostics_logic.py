# Copyright 2024 Roland Arsenault
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""
Pure-Python diagnostic logic for the Teltonika monitor.

Avoids ROS 2 runtime dependencies (rclpy) so it can be unit-tested
against hand-built cache snapshots.  Still imports DiagnosticStatus for
the standard level constants.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


# Task-name suffix builders — centralized so the node and tests agree
# on the exact strings used as dynamic-membership keys.

def mwan3_task_name(name_prefix: str, mwan_name: str) -> str:
    """Build the full task name for a per-mwan3-member diagnostic."""
    return f'{name_prefix}: mwan3/{mwan_name}'


def interface_task_name(name_prefix: str, iface_name: str) -> str:
    """Build the full task name for a per-interface diagnostic."""
    return f'{name_prefix}: interface/{iface_name}'


# System-board fields to surface as KeyValue pairs.
_SYSTEM_FIELDS = ('model', 'hostname', 'kernel', 'system')
_SYSTEM_RELEASE_FIELDS = ('distribution', 'version', 'description')

# Cellular fields.
_CELLULAR_FIELDS = ('net_mode', 'rssi', 'rsrp', 'sinr', 'rsrq')

# mwan3 fields.
_MWAN3_FIELDS = (
    'status', 'online', 'offline', 'uptime',
    'score', 'lost', 'enabled', 'running', 'up',
)

# Network interface fields.
_INTERFACE_FIELDS = (
    'up', 'proto', 'device', 'metric',
    'uptime', 'l3_device',
)


@dataclass
class CachedStatus:
    """
    Latest parsed router response plus metadata.

    The poll callback replaces this whole-dataclass under a lock so task
    callbacks on the rclpy thread only ever see a consistent snapshot.
    ``poll_monotonic == 0.0`` means "no poll has completed yet."
    """

    system_board: Optional[dict] = None          # from /system/board
    cellular_signal: Optional[dict] = None       # from /signal/get (or None if unsupported)
    cellular_supported: bool = True              # False if ubus reported "method not found"
    mwan3: Optional[dict] = None                 # from mwan3 status (or None if unsupported)
    mwan3_supported: bool = True                 # False if ubus reported "method not found"
    network_interfaces: list[dict] = field(default_factory=list)  # from /network/interface/dump
    poll_monotonic: float = 0.0
    poll_wall_iso: str = ''
    error_message: Optional[str] = None  # non-None if last poll attempt failed


def parse_dbm(value) -> Optional[float]:
    """Parse a dBm string like '-85 dBm' to a float, or return None."""
    if value is None:
        return None
    try:
        return float(str(value).split()[0])
    except (ValueError, IndexError):
        return None


def parse_db(value) -> Optional[float]:
    """Parse a dB string like '12.5 dB' to a float, or return None."""
    if value is None:
        return None
    try:
        return float(str(value).split()[0])
    except (ValueError, IndexError):
        return None


def cellular_quality(
    rsrp: Optional[float],
    sinr: Optional[float],
) -> tuple[int, str]:
    """
    Map RSRP/SINR to a diagnostic level and a signal-bars string.

    Thresholds based on 3GPP signal quality ranges for LTE:
      Excellent: RSRP > -80   SINR > 20
      Good:      RSRP > -90   SINR > 13
      Fair:      RSRP > -100  SINR > 0
      Poor:      RSRP > -110  SINR > -5
      Very poor: below
    """
    bar_chars = ['▁', '▂', '▃', '▅', '█']
    if rsrp is None:
        return DiagnosticStatus.WARN, '?'
    if rsrp > -80:
        n = 5
    elif rsrp >= -90:
        n = 4
    elif rsrp >= -100:
        n = 3
    elif rsrp >= -110:
        n = 2
    else:
        n = 1
    # SINR can downgrade by one bar when negative.
    if sinr is not None and sinr < 0 and n > 1:
        n -= 1
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

    Replaces the bug-creating "rename-on-error" status that the
    pre-Updater node emitted.  Name stays fixed as ``<prefix>:
    connection``; level reflects the outcome of the most recent poll
    attempt.

    ``cache.error_message`` is checked first, before staleness, because
    the connection task exists specifically to surface the most recent
    poll attempt's outcome.  That means:

    - On first-poll failure (``poll_monotonic == 0.0`` still at its
      initial default, ``error_message`` set by the preservation path),
      report ERROR with the real reason rather than the generic
      "no successful poll yet" STALE.  The other per-entity tasks still
      emit STALE because they have no cached data to render — that's
      correct for them, but hides the connection error from the
      connection task's consumer if we did the same here.
    - When a formerly-fresh cache ages past ``stale_timeout_sec`` AND
      the most recent attempt errored, the error message is the
      freshest real news, so surface it instead of "cached data Ns old".
    """
    kvs: list[KeyValue] = [_last_query_kv(cache)]

    if cache.error_message:
        return DiagnosticStatus.ERROR, cache.error_message, kvs

    stale_msg = _is_cache_stale(cache, stale_timeout_sec, now_monotonic)
    if stale_msg is not None:
        return DiagnosticStatus.STALE, stale_msg, kvs

    return DiagnosticStatus.OK, 'reachable', kvs


def synthesize_system_status(
    cache: CachedStatus,
    stale_timeout_sec: float,
    now_monotonic: float,
) -> tuple[int, str, list[KeyValue]]:
    """Render the system/board task."""
    kvs: list[KeyValue] = [_last_query_kv(cache)]

    stale_msg = _is_cache_stale(cache, stale_timeout_sec, now_monotonic)
    if stale_msg is not None:
        return DiagnosticStatus.STALE, stale_msg, kvs

    board = cache.system_board
    if board is None:
        return DiagnosticStatus.ERROR, 'system_board not cached', kvs

    for key in _SYSTEM_FIELDS:
        if key in board:
            kvs.append(KeyValue(key=key, value=str(board[key])))

    release = board.get('release', {})
    for key in _SYSTEM_RELEASE_FIELDS:
        if key in release:
            kvs.append(KeyValue(key=f'release.{key}', value=str(release[key])))

    return DiagnosticStatus.OK, board.get('model', 'unknown'), kvs


def synthesize_cellular_status(
    cache: CachedStatus,
    stale_timeout_sec: float,
    now_monotonic: float,
) -> tuple[int, str, list[KeyValue]]:
    """
    Render the cellular-signal task.

    Handles three cases:
    - Router doesn't support cellular (no modem): WARN "cellular not supported"
    - Modem present but no service: WARN "No service"
    - Active service: level derived from RSRP/SINR thresholds
    """
    kvs: list[KeyValue] = [_last_query_kv(cache)]

    stale_msg = _is_cache_stale(cache, stale_timeout_sec, now_monotonic)
    if stale_msg is not None:
        return DiagnosticStatus.STALE, stale_msg, kvs

    if not cache.cellular_supported:
        return DiagnosticStatus.WARN, 'cellular not supported', kvs

    signal = cache.cellular_signal
    if signal is None:
        return DiagnosticStatus.WARN, 'no cellular data', kvs

    net_mode = signal.get('net_mode', 'No service')

    for key in _CELLULAR_FIELDS:
        if key in signal:
            kvs.append(KeyValue(key=key, value=str(signal[key])))

    if net_mode == 'No service':
        return DiagnosticStatus.WARN, 'No service', kvs

    rsrp = parse_dbm(signal.get('rsrp'))
    sinr = parse_db(signal.get('sinr'))
    level, bars = cellular_quality(rsrp, sinr)
    rsrp_str = f' {rsrp:.0f}dBm' if rsrp is not None else ''
    return level, f'{net_mode}{rsrp_str} {bars}', kvs


def synthesize_mwan3_status(
    cache: CachedStatus,
    mwan_name: str,
    stale_timeout_sec: float,
    now_monotonic: float,
) -> tuple[int, str, list[KeyValue]]:
    """Render one per-mwan3-member task."""
    kvs: list[KeyValue] = [_last_query_kv(cache)]

    stale_msg = _is_cache_stale(cache, stale_timeout_sec, now_monotonic)
    if stale_msg is not None:
        return DiagnosticStatus.STALE, stale_msg, kvs

    if not cache.mwan3_supported:
        return (
            DiagnosticStatus.STALE,
            'mwan3 not supported on this router',
            kvs,
        )

    if cache.mwan3 is None:
        # No cached mwan3 data — either never observed, or the most
        # recent poll failed before reaching the mwan3 query.  Surface
        # the connection error if present so the task identity stays
        # intact and the operator sees a member-specific message.
        if cache.error_message:
            return (
                DiagnosticStatus.STALE,
                f'mwan3/{mwan_name}: {cache.error_message}',
                kvs,
            )
        return (
            DiagnosticStatus.STALE,
            f'mwan3 data for {mwan_name} not cached yet',
            kvs,
        )

    interfaces = cache.mwan3.get('interfaces', {})
    data = interfaces.get(mwan_name)
    if data is None:
        return (
            DiagnosticStatus.STALE,
            f'mwan3 member {mwan_name!r} not in latest poll',
            kvs,
        )

    wan_status = data.get('status', 'unknown')
    if wan_status == 'online':
        level, message = DiagnosticStatus.OK, 'Online'
    elif wan_status == 'standby':
        # Standby is the expected steady state for a backup interface
        # while the primary is up — not a warning.
        level, message = DiagnosticStatus.OK, 'Standby'
    elif wan_status == 'offline':
        level, message = DiagnosticStatus.ERROR, 'Offline'
    else:
        level, message = DiagnosticStatus.WARN, wan_status

    for key in _MWAN3_FIELDS:
        if key in data:
            kvs.append(KeyValue(key=key, value=str(data[key])))

    # Track-IP health sub-fields
    for track in data.get('track_ip', []):
        ip = track.get('ip', '?')
        track_status = track.get('status', '?')
        latency = track.get('latency', 0)
        loss = track.get('packetloss', 0)
        kvs.append(KeyValue(
            key=f'track/{ip}',
            value=f'{track_status} latency={latency} loss={loss}',
        ))

    return level, message, kvs


def synthesize_interface_status(
    cache: CachedStatus,
    iface_name: str,
    stale_timeout_sec: float,
    now_monotonic: float,
) -> tuple[int, str, list[KeyValue]]:
    """Render one per-network-interface task."""
    kvs: list[KeyValue] = [_last_query_kv(cache)]

    stale_msg = _is_cache_stale(cache, stale_timeout_sec, now_monotonic)
    if stale_msg is not None:
        return DiagnosticStatus.STALE, stale_msg, kvs

    iface = _find_interface(cache.network_interfaces, iface_name)
    if iface is None:
        return (
            DiagnosticStatus.STALE,
            f'interface {iface_name!r} not in latest poll',
            kvs,
        )

    is_up = iface.get('up', False)
    if is_up:
        level, message = DiagnosticStatus.OK, 'Up'
    else:
        level, message = DiagnosticStatus.WARN, 'Down'

    for key in _INTERFACE_FIELDS:
        if key in iface:
            kvs.append(KeyValue(key=key, value=str(iface[key])))

    for addr_info in iface.get('ipv4-address', []):
        addr = addr_info.get('address', '')
        mask = addr_info.get('mask', '')
        if addr:
            kvs.append(KeyValue(key='ipv4', value=f'{addr}/{mask}'))

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

    Identical contract to ``mikrotik_monitor.diagnostics_logic``: returns
    ``(to_add, to_remove, new_last_seen)``.  Duplicated (rather than
    shared) because the two monitor packages don't otherwise share
    modules — keep each package standalone for simpler install-path
    reasoning.
    """
    to_add = observed - registered
    new_last_seen: dict[str, float] = {}
    to_remove: set[str] = set()

    for name in registered | observed:
        if name in observed:
            new_last_seen[name] = now_monotonic
        else:
            previous = last_seen.get(name, now_monotonic)
            if now_monotonic - previous > grace_sec:
                to_remove.add(name)
            else:
                new_last_seen[name] = previous

    return to_add, to_remove, new_last_seen


def _find_interface(items: list[dict], iface_name: str) -> Optional[dict]:
    """
    Locate an interface by its 'interface' key.

    Teltonika's ubus network/interface/dump returns each entry with the
    logical name under the 'interface' key rather than 'name' (unlike
    the RouterOS convention used by mikrotik_monitor).
    """
    for item in items:
        if item.get('interface') == iface_name:
            return item
    return None
