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
from datetime import datetime, timedelta, timezone
import re
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


def wireless_events_task_name(name_prefix: str, iface: str) -> str:
    """Build the full task name for a per-radio wireless-events diagnostic."""
    return f'{name_prefix}: events/{iface}'


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
    wireless_events: list[dict] = field(default_factory=list)  # from /log filtered to wireless topics
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
    reflects the outcome of the most recent poll attempt.

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


# ---------------------------------------------------------------------------
# Wireless-event log surfacing (see ros2_network_monitor#21)
# ---------------------------------------------------------------------------
#
# RouterOS wireless log messages have a consistent "MAC@interface" anchor,
# e.g. "C4:AD:34:90:72:4C@wlan1: lost connection, extensive data loss" or
# "C4:AD:34:90:72:4C@wlan1 established connection on 5745000, SSID
# bizzy_bridge". Extract the interface for per-radio scoping.

_IFACE_AT_RE = re.compile(r'@([A-Za-z0-9_-]+)[\s:]')


def event_interface(event: dict) -> Optional[str]:
    """Extract the RouterOS interface name from a wireless log event.

    Returns None if no ``@interface`` anchor is present in the message.
    """
    msg = event.get('message') or ''
    match = _IFACE_AT_RE.search(msg)
    return match.group(1) if match else None


def classify_event(message: str) -> Optional[str]:
    """Classify a wireless log message as ``'assoc'`` or ``'drop'``.

    Returns None for messages that don't fit either category (other
    diagnostics-level wireless chatter we don't care about for session
    accounting).
    """
    if not message:
        return None
    if 'established connection' in message:
        return 'assoc'
    if 'lost connection' in message or 'deauth' in message:
        return 'drop'
    return None


# RouterOS log time formats observed in the field.  Tried in order.
_LOG_TIME_FORMATS = (
    '%Y-%m-%d %H:%M:%S',     # 2026-04-14 09:52:22 (uptime-based, after sync)
    '%b/%d/%Y %H:%M:%S',     # Mar/09/2026 09:18:05 (after time-zone-name set)
)


def parse_log_time(time_str: str) -> Optional[datetime]:
    """Parse a RouterOS log ``time`` field into a UTC datetime.

    Returns None on parse failure.  Assumes UTC because we configure the
    bridges with ``time-zone-name=UTC``.  If a deployment ever uses a
    local time zone, this would need a per-device tz parameter.

    The ``%H:%M:%S``-only short form (which RouterOS emits for entries
    whose date matches "today") is intentionally NOT supported here:
    without a date the entry's wall time is ambiguous, and downstream
    code can't compute rolling-window membership reliably.  In practice
    the wireless events we care about either fall in the same day as
    "now" (where the short form would work but we'd need extra logic)
    or are older (where the short form would silently break).  Treat
    short-form entries as unparseable and drop them.
    """
    if not time_str:
        return None
    for fmt in _LOG_TIME_FORMATS:
        try:
            dt = datetime.strptime(time_str, fmt)
            return dt.replace(tzinfo=timezone.utc)
        except ValueError:
            continue
    return None


def synthesize_wireless_events_status(
    cache: CachedStatus,
    iface: str,
    stale_timeout_sec: float,
    now_monotonic: float,
    now_wall_dt: Optional[datetime] = None,
) -> tuple[int, str, list[KeyValue]]:
    """Render the per-radio wireless-events task.

    Surfaces association / drop / deauth event metrics from RouterOS's
    log buffer:

    - ``last_assoc`` — wall-time and "Ns ago" of the most recent
      ``established connection`` event for this radio.
    - ``last_drop`` — wall-time, "Ns ago", and reason text of the most
      recent ``lost connection`` / deauth event.
    - ``drops_last_5min`` / ``drops_last_60min`` — rolling counters.
    - ``current_session_age_sec`` — seconds since the most recent
      ``assoc`` event, IF no later ``drop`` exists.  ``no_active_session``
      otherwise.

    Severity is **always OK** as long as the underlying poll is healthy.
    Wireless drops are routine for marine ops (over-horizon / out-of-range
    is a valid mode of operation; see workspace feedback memory
    ``feedback_wifi_disconnect_not_an_error``).  Downstream consumers
    (annunciators with range awareness, bag-time forensics) can apply
    context to decide whether a drop pattern is concerning.
    """
    kvs: list[KeyValue] = [_last_query_kv(cache)]

    stale_msg = _is_cache_stale(cache, stale_timeout_sec, now_monotonic)
    if stale_msg is not None:
        return DiagnosticStatus.STALE, stale_msg, kvs

    if now_wall_dt is None:
        now_wall_dt = datetime.now(timezone.utc)

    # Parse + classify + filter to this interface
    iface_events: list[tuple[datetime, str, str]] = []   # (when, kind, msg)
    for event in cache.wireless_events:
        if event_interface(event) != iface:
            continue
        kind = classify_event(event.get('message') or '')
        if kind is None:
            continue
        when = parse_log_time(event.get('time') or '')
        if when is None:
            continue
        iface_events.append((when, kind, event.get('message') or ''))

    # Sort by time so "last" is reliable regardless of RouterOS's
    # log-ordering convention.  RouterOS emits log in insertion order
    # (oldest first); sorting defensively makes the function robust to
    # future reorderings.
    iface_events.sort(key=lambda triple: triple[0])

    # Most-recent assoc and drop
    last_assoc_ts: Optional[datetime] = None
    last_drop_ts: Optional[datetime] = None
    last_drop_msg: Optional[str] = None
    for ts, kind, msg in iface_events:
        if kind == 'assoc':
            last_assoc_ts = ts
        elif kind == 'drop':
            last_drop_ts = ts
            last_drop_msg = msg

    # Rolling counters
    five_min_ago = now_wall_dt - timedelta(minutes=5)
    sixty_min_ago = now_wall_dt - timedelta(minutes=60)
    drops_5min = sum(
        1 for ts, kind, _ in iface_events
        if kind == 'drop' and ts >= five_min_ago
    )
    drops_60min = sum(
        1 for ts, kind, _ in iface_events
        if kind == 'drop' and ts >= sixty_min_ago
    )

    # Current session age: time since last_assoc, only if no later drop.
    current_session_age_sec: Optional[int] = None
    if last_assoc_ts is not None and (
        last_drop_ts is None or last_assoc_ts > last_drop_ts
    ):
        current_session_age_sec = int(
            (now_wall_dt - last_assoc_ts).total_seconds()
        )

    # KeyValue output
    if last_assoc_ts is not None:
        ago = int((now_wall_dt - last_assoc_ts).total_seconds())
        kvs.append(KeyValue(
            key='last_assoc',
            value=f'{ago}s ago ({last_assoc_ts.isoformat()})',
        ))
    else:
        kvs.append(KeyValue(
            key='last_assoc',
            value='never (or older than log buffer)',
        ))

    if last_drop_ts is not None:
        ago = int((now_wall_dt - last_drop_ts).total_seconds())
        kvs.append(KeyValue(
            key='last_drop',
            value=f'{ago}s ago: {last_drop_msg}',
        ))
    else:
        kvs.append(KeyValue(
            key='last_drop',
            value='never (or older than log buffer)',
        ))

    kvs.append(KeyValue(key='drops_last_5min', value=str(drops_5min)))
    kvs.append(KeyValue(key='drops_last_60min', value=str(drops_60min)))

    if current_session_age_sec is not None:
        kvs.append(KeyValue(
            key='current_session_age_sec',
            value=str(current_session_age_sec),
        ))
    else:
        kvs.append(KeyValue(
            key='current_session_age_sec',
            value='no_active_session',
        ))

    # Short human message — drops in last hour are the most actionable
    # at-a-glance signal.  Note: this is descriptive, not alarming
    # (level stays OK regardless of count).
    if drops_5min == 0 and drops_60min == 0:
        message = 'No drops in last hour'
    else:
        message = f'{drops_5min} drops in 5min, {drops_60min} in 60min'

    return DiagnosticStatus.OK, message, kvs
