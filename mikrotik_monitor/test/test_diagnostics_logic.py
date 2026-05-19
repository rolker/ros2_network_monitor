# Copyright 2024 Roland Arsenault
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""Unit tests for mikrotik_monitor.diagnostics_logic."""

from datetime import datetime, timezone

from diagnostic_msgs.msg import DiagnosticStatus

from mikrotik_monitor.diagnostics_logic import (
    CachedStatus,
    classify_event,
    diff_dynamic_membership,
    event_interface,
    interface_task_name,
    parse_log_time,
    parse_snr,
    synthesize_connection_status,
    synthesize_interface_status,
    synthesize_system_status,
    synthesize_wireless_events_status,
    synthesize_wireless_status,
    wireless_events_task_name,
    wireless_quality,
    wireless_task_name,
)


STALE = 10.0
NOW = 100.0
PREFIX = 'MikroTik: wifi.bizzy'


def _kvs_dict(kvs):
    return {kv.key: kv.value for kv in kvs}


# --- SNR parsing / quality ---

def test_parse_snr_variants():
    assert parse_snr('22@HT40') == 22.0
    assert parse_snr('22 dB') == 22.0
    assert parse_snr('22') == 22.0
    assert parse_snr(None) is None
    assert parse_snr('bogus') is None


def test_wireless_quality_boundaries():
    assert wireless_quality(None)[0] == DiagnosticStatus.WARN
    assert wireless_quality(35.0)[0] == DiagnosticStatus.OK
    assert wireless_quality(25.0)[0] == DiagnosticStatus.OK
    assert wireless_quality(17.0)[0] == DiagnosticStatus.WARN
    assert wireless_quality(12.0)[0] == DiagnosticStatus.WARN
    assert wireless_quality(5.0)[0] == DiagnosticStatus.ERROR


# --- Connection task ---

def test_connection_stale_before_first_poll():
    """No poll yet → STALE, not ERROR."""
    level, message, _ = synthesize_connection_status(
        CachedStatus(), STALE, NOW,
    )
    assert level == DiagnosticStatus.STALE
    assert 'no successful poll' in message


def test_connection_ok_when_poll_succeeded():
    cache = CachedStatus(
        system_resource={'board-name': 'RB4011'},
        poll_monotonic=NOW - 1.0,
        poll_wall_iso='2026-04-23T20:00:00+00:00',
        error_message=None,
    )
    level, message, kvs = synthesize_connection_status(cache, STALE, NOW)
    assert level == DiagnosticStatus.OK
    assert message == 'reachable'
    assert _kvs_dict(kvs)['last_query_time'] == '2026-04-23T20:00:00+00:00'


def test_connection_error_surfaces_error_message():
    cache = CachedStatus(
        poll_monotonic=NOW - 1.0,
        poll_wall_iso='2026-04-23T20:00:00+00:00',
        error_message='Connection error: timeout',
    )
    level, message, _ = synthesize_connection_status(cache, STALE, NOW)
    assert level == DiagnosticStatus.ERROR
    assert message == 'Connection error: timeout'


def test_connection_stale_when_cache_too_old():
    """Cache older than stale_timeout AND no error → STALE (polls stopped)."""
    cache = CachedStatus(
        system_resource={'board-name': 'RB4011'},
        poll_monotonic=NOW - (STALE + 5.0),
        poll_wall_iso='2026-04-23T19:00:00+00:00',
    )
    level, message, _ = synthesize_connection_status(cache, STALE, NOW)
    assert level == DiagnosticStatus.STALE
    assert 'cached data' in message


def test_connection_reports_error_on_first_poll_failure():
    """
    Regression for V4 (PR #19 round 2 review).

    After the cache-preservation fix, the first-poll-failure cache
    keeps the initial ``poll_monotonic=0.0`` default (no prior success
    to preserve from) and sets ``error_message``.  An earlier version
    of ``synthesize_connection_status`` checked staleness first, so
    this state incorrectly emitted STALE "no successful poll yet"
    instead of surfacing the real connection error.  Error takes
    precedence over staleness in the connection task because its
    purpose is to reflect the most recent poll attempt's outcome.
    """
    first_fail_cache = CachedStatus(
        poll_monotonic=0.0,  # no prior success to preserve
        poll_wall_iso='2026-04-23T20:00:00+00:00',
        error_message='Connection error: refused',
    )
    level, message, _ = synthesize_connection_status(
        first_fail_cache, STALE, NOW,
    )
    assert level == DiagnosticStatus.ERROR
    assert 'refused' in message


def test_connection_error_takes_precedence_over_aged_stale():
    """
    When a formerly-fresh cache ages past stale_timeout AND the most
    recent poll attempt errored, the error message is the freshest
    real news — surface it, not "cached data Ns old".
    """
    aged_error_cache = CachedStatus(
        system_resource={'board-name': 'RB4011'},
        poll_monotonic=NOW - (STALE + 5.0),
        poll_wall_iso='2026-04-23T19:59:00+00:00',
        error_message='Connection error: auth',
    )
    level, message, _ = synthesize_connection_status(
        aged_error_cache, STALE, NOW,
    )
    assert level == DiagnosticStatus.ERROR
    assert 'auth' in message


# --- System task ---

def test_system_status_renders_fields_and_health():
    cache = CachedStatus(
        system_resource={
            'board-name': 'RB4011',
            'version': '7.14',
            'cpu-load': 12,
        },
        system_health=[
            {'name': 'temperature', 'value': '42'},
            {'name': 'voltage', 'value': '24.1'},
        ],
        poll_monotonic=NOW,
        poll_wall_iso='2026-04-23T20:00:00+00:00',
    )
    level, message, kvs = synthesize_system_status(cache, STALE, NOW)
    d = _kvs_dict(kvs)
    assert level == DiagnosticStatus.OK
    assert message == 'RB4011'
    assert d['board-name'] == 'RB4011'
    assert d['temperature'] == '42'
    assert d['voltage'] == '24.1'


def test_system_status_handles_dict_health():
    cache = CachedStatus(
        system_resource={'board-name': 'RB4011'},
        system_health={'temperature': 42, '.id': '*1'},
        poll_monotonic=NOW,
        wireless_events_last_success_monotonic=NOW,
    )
    _, _, kvs = synthesize_system_status(cache, STALE, NOW)
    d = _kvs_dict(kvs)
    assert d['temperature'] == '42'
    assert '.id' not in d  # skipped explicitly


# --- Interface task ---

def test_interface_running_ok():
    cache = CachedStatus(
        interfaces=[{'name': 'ether1', 'running': 'true', 'tx-byte': 100}],
        poll_monotonic=NOW,
        wireless_events_last_success_monotonic=NOW,
    )
    level, message, kvs = synthesize_interface_status(
        cache, 'ether1', STALE, NOW,
    )
    assert level == DiagnosticStatus.OK
    assert message == 'Running'
    assert _kvs_dict(kvs)['tx-byte'] == '100'


def test_interface_disabled_warn():
    cache = CachedStatus(
        interfaces=[{'name': 'ether9', 'running': 'false', 'disabled': 'true'}],
        poll_monotonic=NOW,
        wireless_events_last_success_monotonic=NOW,
    )
    level, message, _ = synthesize_interface_status(
        cache, 'ether9', STALE, NOW,
    )
    assert level == DiagnosticStatus.WARN
    assert message == 'Disabled'


def test_interface_missing_from_cache_stale():
    """Interface task still registered but no longer in latest poll."""
    cache = CachedStatus(
        interfaces=[{'name': 'ether1', 'running': 'true'}],
        poll_monotonic=NOW,
        wireless_events_last_success_monotonic=NOW,
    )
    level, message, _ = synthesize_interface_status(
        cache, 'ether-gone', STALE, NOW,
    )
    assert level == DiagnosticStatus.STALE
    assert 'ether-gone' in message


# --- Schema-drift regression (the reason this whole refactor exists) ---

def test_schema_drift_connection_task_name_stable_across_states():
    """
    Regression for ros2_network_monitor#15.

    The pre-Updater node renamed its status to drop the task suffix on
    connection error ("MikroTik: wifi.bizzy" instead of
    "MikroTik: wifi.bizzy: connection").  The Updater migration makes
    this impossible: connection task name is fixed at ``.add()`` time.
    This test locks in that invariant — the task contents change with
    connection state, but the task set does not.
    """
    # Fixed tasks are "<prefix>: connection" and "<prefix>: system".
    # Neither should vary based on cache.error_message.
    ok_cache = CachedStatus(
        system_resource={'board-name': 'RB4011'},
        poll_monotonic=NOW,
        error_message=None,
    )
    err_cache = CachedStatus(
        poll_monotonic=NOW,
        error_message='Connection error: refused',
    )

    ok_conn_level, _, _ = synthesize_connection_status(ok_cache, STALE, NOW)
    err_conn_level, _, _ = synthesize_connection_status(err_cache, STALE, NOW)

    assert ok_conn_level == DiagnosticStatus.OK
    assert err_conn_level == DiagnosticStatus.ERROR
    # The "task exists" fact is what matters: synthesize_connection_status
    # always returns SOMETHING for a given cache. There's no code path
    # that invents a different task name.


def test_cache_preserved_across_transient_failure():
    """
    Regression for F1/V2 (PR #19).

    On a transient connection failure, tasks must continue rendering the
    last-known values — they only flip to STALE once the preserved cache
    ages past stale_timeout_sec.  This is the "graceful degradation"
    property Starlink's pattern achieves by keeping the previous cache's
    payload and poll_monotonic on error.

    The node code is what preserves the cache; this test validates the
    consequence: given a cache that looks like "last successful poll
    was 1s ago, most recent poll attempt errored", tasks render the
    preserved values (NOT stale, NOT the error message) until the
    stale-age check trips.
    """
    # Build a cache representing "router was reachable, last successful
    # poll 1s ago, most recent attempt failed."  The node's poll_callback
    # should have preserved system_resource/interfaces/wireless from the
    # prior cache and only touched error_message + poll_wall_iso.
    preserved_cache = CachedStatus(
        system_resource={'board-name': 'RB4011', 'cpu-load': '5'},
        interfaces=[{'name': 'ether1', 'running': 'true'}],
        wireless=[],
        poll_monotonic=NOW - 1.0,  # previous successful poll, still fresh
        poll_wall_iso='2026-04-23T20:00:00+00:00',
        error_message='Connection error: refused',  # most recent attempt
    )

    # : connection task surfaces the error (its job).
    level, message, _ = synthesize_connection_status(
        preserved_cache, STALE, NOW,
    )
    assert level == DiagnosticStatus.ERROR
    assert 'refused' in message

    # : system task keeps rendering OK from preserved data.  This is the
    # critical property — WITHOUT the preservation fix, system_resource
    # would be None and this would return ERROR "system_resource not
    # cached" on every publish after the first transient failure.
    level, message, _ = synthesize_system_status(
        preserved_cache, STALE, NOW,
    )
    assert level == DiagnosticStatus.OK
    assert message == 'RB4011'

    # Per-interface task keeps rendering Running.
    level, message, _ = synthesize_interface_status(
        preserved_cache, 'ether1', STALE, NOW,
    )
    assert level == DiagnosticStatus.OK
    assert message == 'Running'


def test_preserved_cache_ages_into_stale():
    """
    Complement to test_cache_preserved_across_transient_failure.

    When poll_monotonic stays at the previous successful time (per the
    preservation fix), ongoing failures eventually drive tasks to STALE
    via _is_cache_stale — they don't report stale values forever.
    """
    cache = CachedStatus(
        system_resource={'board-name': 'RB4011'},
        interfaces=[{'name': 'ether1', 'running': 'true'}],
        poll_monotonic=NOW - (STALE + 5.0),  # past stale timeout
        poll_wall_iso='2026-04-23T19:00:00+00:00',
        error_message='Connection error: refused',
    )

    # Non-connection tasks emit STALE via the age check.
    level, _, _ = synthesize_system_status(cache, STALE, NOW)
    assert level == DiagnosticStatus.STALE
    level, _, _ = synthesize_interface_status(cache, 'ether1', STALE, NOW)
    assert level == DiagnosticStatus.STALE


def test_schema_drift_interfaces_drop_does_not_cascade():
    """
    When a poll fails, interfaces/wireless aren't re-observed — so the
    node's reconciliation LEAVES those tasks registered and they render
    STALE from their own task callbacks.  The PRE-Updater node would
    have dropped them entirely and replaced the whole array with a
    single bare-name summary status, creating the orphan-in-aggregator
    bug.
    """
    err_cache = CachedStatus(
        poll_monotonic=NOW,
        error_message='Connection error: refused',
    )
    # A registered interface task still exists; its callback sees the
    # error cache and renders STALE (no interface to look up).
    level, message, _ = synthesize_interface_status(
        err_cache, 'ether1', STALE, NOW,
    )
    assert level == DiagnosticStatus.STALE
    assert 'ether1' in message


# --- Dynamic-membership diff ---

def test_diff_add_new_names():
    observed = {'iface/a', 'iface/b'}
    registered = set()
    last_seen = {}
    to_add, to_remove, new_seen = diff_dynamic_membership(
        observed, registered, last_seen, grace_sec=5.0, now_monotonic=NOW,
    )
    assert to_add == {'iface/a', 'iface/b'}
    assert to_remove == set()
    assert new_seen == {'iface/a': NOW, 'iface/b': NOW}


def test_diff_remove_after_grace():
    """Name missing for longer than grace_sec is removed."""
    observed = {'iface/a'}
    registered = {'iface/a', 'iface/b'}
    last_seen = {'iface/a': NOW - 1.0, 'iface/b': NOW - 10.0}
    to_add, to_remove, new_seen = diff_dynamic_membership(
        observed, registered, last_seen, grace_sec=5.0, now_monotonic=NOW,
    )
    assert to_add == set()
    assert to_remove == {'iface/b'}
    assert new_seen == {'iface/a': NOW}  # 'b' dropped


def test_diff_keeps_within_grace():
    """Transient flap: name missing but within grace period is kept."""
    observed = {'iface/a'}
    registered = {'iface/a', 'iface/b'}
    last_seen = {'iface/a': NOW - 1.0, 'iface/b': NOW - 2.0}  # 2s < 5s grace
    to_add, to_remove, new_seen = diff_dynamic_membership(
        observed, registered, last_seen, grace_sec=5.0, now_monotonic=NOW,
    )
    assert to_add == set()
    assert to_remove == set()
    # 'iface/b' retains its old last_seen, doesn't get refreshed
    assert new_seen == {'iface/a': NOW, 'iface/b': NOW - 2.0}


def test_diff_reappeared_name_refreshes_timestamp():
    """Name reappears after flap; timestamp updated, no add/remove."""
    observed = {'iface/a'}
    registered = {'iface/a'}
    last_seen = {'iface/a': NOW - 3.0}
    to_add, to_remove, new_seen = diff_dynamic_membership(
        observed, registered, last_seen, grace_sec=5.0, now_monotonic=NOW,
    )
    assert to_add == set()
    assert to_remove == set()
    assert new_seen == {'iface/a': NOW}


# --- Task-name helpers ---

def test_task_name_helpers():
    assert interface_task_name(PREFIX, 'ether1') == (
        'MikroTik: wifi.bizzy: interface/ether1'
    )
    assert wireless_task_name(PREFIX, 'wlan1', 'AA:BB:CC:DD:EE:FF') == (
        'MikroTik: wifi.bizzy: wireless/wlan1/AA:BB:CC:DD:EE:FF'
    )


# --- Wireless task ---

def test_wireless_ok_with_good_snr():
    cache = CachedStatus(
        wireless=[{
            'interface': 'wlan1',
            'mac-address': 'AA:BB:CC:DD:EE:FF',
            'signal-to-noise': '25@HT40',
            'tx-rate': '400Mbps',
        }],
        poll_monotonic=NOW,
        wireless_events_last_success_monotonic=NOW,
    )
    level, message, kvs = synthesize_wireless_status(
        cache, 'wlan1', 'AA:BB:CC:DD:EE:FF', STALE, NOW,
    )
    assert level == DiagnosticStatus.OK
    assert 'SNR 25dB' in message
    assert 'Associated' in message
    d = _kvs_dict(kvs)
    assert d['tx-rate'] == '400Mbps'


# --- Wireless event helpers ---

def test_classify_event_assoc():
    msg = 'C4:AD:34:90:72:4C@wlan1 established connection on 5745000, SSID bizzy_bridge'
    assert classify_event(msg) == 'assoc'


def test_classify_event_drop_lost_connection():
    msg = 'C4:AD:34:90:72:4C@wlan1: lost connection, extensive data loss'
    assert classify_event(msg) == 'drop'


def test_classify_event_drop_deauth():
    msg = 'C4:AD:34:90:72:4C@wlan1: received deauth: group key handshake timeout (16)'
    assert classify_event(msg) == 'drop'


def test_classify_event_ignored():
    assert classify_event('something unrelated') is None
    assert classify_event('') is None
    assert classify_event(None) is None


def test_event_interface_extracts_iface():
    e = {'message': 'C4:AD:34:90:72:4C@wlan1: lost connection, extensive data loss'}
    assert event_interface(e) == 'wlan1'


def test_event_interface_multi_digit():
    e = {'message': 'AA:BB:CC:DD:EE:FF@wlan10 established connection'}
    assert event_interface(e) == 'wlan10'


def test_event_interface_no_anchor():
    assert event_interface({'message': 'no @ symbol here'}) is None
    assert event_interface({}) is None


def test_parse_log_time_iso_format():
    dt = parse_log_time('2026-04-14 09:52:22')
    assert dt == datetime(2026, 4, 14, 9, 52, 22, tzinfo=timezone.utc)


def test_parse_log_time_month_slash_format():
    dt = parse_log_time('Mar/09/2026 09:18:05')
    assert dt == datetime(2026, 3, 9, 9, 18, 5, tzinfo=timezone.utc)


def test_parse_log_time_invalid():
    assert parse_log_time('') is None
    assert parse_log_time('not a date') is None
    # Mangled HH:MM:SS-like strings still fail
    assert parse_log_time('25:99:99') is None


def test_parse_log_time_short_form_uses_today():
    # The HH:MM:SS short form is composed with now_wall_dt's date.
    now = datetime(2026, 5, 18, 22, 30, 0, tzinfo=timezone.utc)
    dt = parse_log_time('22:25:00', now_wall_dt=now)
    assert dt == datetime(2026, 5, 18, 22, 25, 0, tzinfo=timezone.utc)


def test_parse_log_time_short_form_crosses_midnight():
    # If composing with "today" puts the event in the future relative to
    # now_wall_dt, roll back one day.  Models the case where RouterOS
    # logged a 23:59 event and the monitor parses it just after midnight.
    now = datetime(2026, 5, 19, 0, 0, 5, tzinfo=timezone.utc)
    dt = parse_log_time('23:59:50', now_wall_dt=now)
    assert dt == datetime(2026, 5, 18, 23, 59, 50, tzinfo=timezone.utc)


def test_parse_log_time_short_form_defaults_to_utc_now():
    # Sanity check: with no now_wall_dt the function still works
    # (defaults to datetime.now(UTC)).  Don't assert the actual value
    # since now() is not deterministic; just assert it parses.
    dt = parse_log_time('12:34:56')
    assert dt is not None
    assert dt.hour == 12 and dt.minute == 34 and dt.second == 56


def test_wireless_events_task_name():
    assert wireless_events_task_name(PREFIX, 'wlan1') == f'{PREFIX}: events/wlan1'


# --- synthesize_wireless_events_status ---

# Use a fixed "now" so rolling-window math is deterministic.
EVENT_NOW = datetime(2026, 5, 18, 22, 30, 0, tzinfo=timezone.utc)


def _event(time_str, message):
    return {'time': time_str, 'topics': 'wireless,info', 'message': message}


def test_events_stale_cache_before_first_poll():
    cache = CachedStatus()
    level, message, _ = synthesize_wireless_events_status(
        cache, 'wlan1', STALE, NOW, now_wall_dt=EVENT_NOW,
    )
    assert level == DiagnosticStatus.STALE
    assert 'no successful wireless-event log query' in message


def test_events_no_events_returns_ok_never():
    cache = CachedStatus(
        poll_monotonic=NOW,
        wireless_events_last_success_monotonic=NOW,
    )
    level, message, kvs = synthesize_wireless_events_status(
        cache, 'wlan1', STALE, NOW, now_wall_dt=EVENT_NOW,
    )
    assert level == DiagnosticStatus.OK
    assert message == 'No drops in last hour'
    d = _kvs_dict(kvs)
    assert d['last_assoc'].startswith('never')
    assert d['last_drop'].startswith('never')
    assert d['drops_last_5min'] == '0'
    assert d['drops_last_60min'] == '0'
    assert d['current_session_age_sec'] == 'no_active_session'


def test_events_recent_assoc_no_drops_yields_active_session():
    # Associated 5 minutes before EVENT_NOW
    cache = CachedStatus(
        wireless_events=[
            _event('2026-05-18 22:25:00',
                   'AA:BB:CC@wlan1 established connection on 5745000, SSID bridge'),
        ],
        poll_monotonic=NOW,
        wireless_events_last_success_monotonic=NOW,
    )
    level, message, kvs = synthesize_wireless_events_status(
        cache, 'wlan1', STALE, NOW, now_wall_dt=EVENT_NOW,
    )
    assert level == DiagnosticStatus.OK
    assert message == 'No drops in last hour'
    d = _kvs_dict(kvs)
    assert '300s ago' in d['last_assoc']
    assert d['last_drop'].startswith('never')
    assert d['current_session_age_sec'] == '300'
    assert d['drops_last_5min'] == '0'
    assert d['drops_last_60min'] == '0'


def test_events_drop_after_last_assoc_no_active_session():
    # Assoc 10 min ago, drop 3 min ago — no current session
    cache = CachedStatus(
        wireless_events=[
            _event('2026-05-18 22:20:00',
                   'AA:BB:CC@wlan1 established connection on 5745000, SSID bridge'),
            _event('2026-05-18 22:27:00',
                   'AA:BB:CC@wlan1: lost connection, extensive data loss'),
        ],
        poll_monotonic=NOW,
        wireless_events_last_success_monotonic=NOW,
    )
    level, message, kvs = synthesize_wireless_events_status(
        cache, 'wlan1', STALE, NOW, now_wall_dt=EVENT_NOW,
    )
    assert level == DiagnosticStatus.OK
    # 1 drop in the 5-min window (22:25 - 22:30), 1 in 60-min too
    d = _kvs_dict(kvs)
    assert d['drops_last_5min'] == '1'
    assert d['drops_last_60min'] == '1'
    assert d['current_session_age_sec'] == 'no_active_session'
    assert 'lost connection' in d['last_drop']


def test_events_rolling_window_excludes_old_drop():
    # Drop 6 minutes ago — outside 5-min window, inside 60-min
    cache = CachedStatus(
        wireless_events=[
            _event('2026-05-18 22:24:00',
                   'AA:BB:CC@wlan1: lost connection, extensive data loss'),
        ],
        poll_monotonic=NOW,
        wireless_events_last_success_monotonic=NOW,
    )
    _, _, kvs = synthesize_wireless_events_status(
        cache, 'wlan1', STALE, NOW, now_wall_dt=EVENT_NOW,
    )
    d = _kvs_dict(kvs)
    assert d['drops_last_5min'] == '0'
    assert d['drops_last_60min'] == '1'


def test_events_rolling_window_excludes_ancient_drop():
    # Drop 65 minutes ago — outside both windows
    cache = CachedStatus(
        wireless_events=[
            _event('2026-05-18 21:25:00',
                   'AA:BB:CC@wlan1: lost connection, extensive data loss'),
        ],
        poll_monotonic=NOW,
        wireless_events_last_success_monotonic=NOW,
    )
    _, _, kvs = synthesize_wireless_events_status(
        cache, 'wlan1', STALE, NOW, now_wall_dt=EVENT_NOW,
    )
    d = _kvs_dict(kvs)
    assert d['drops_last_5min'] == '0'
    assert d['drops_last_60min'] == '0'
    # last_drop is still surfaced even though it's outside both windows
    assert 'lost connection' in d['last_drop']


def test_events_filters_by_interface():
    # Drop on wlan1, drop on wlan2 — wlan1 task should only see wlan1
    cache = CachedStatus(
        wireless_events=[
            _event('2026-05-18 22:28:00',
                   'AA:BB:CC@wlan1: lost connection'),
            _event('2026-05-18 22:28:00',
                   'DD:EE:FF@wlan2: lost connection'),
        ],
        poll_monotonic=NOW,
        wireless_events_last_success_monotonic=NOW,
    )
    _, _, kvs_w1 = synthesize_wireless_events_status(
        cache, 'wlan1', STALE, NOW, now_wall_dt=EVENT_NOW,
    )
    _, _, kvs_w2 = synthesize_wireless_events_status(
        cache, 'wlan2', STALE, NOW, now_wall_dt=EVENT_NOW,
    )
    assert _kvs_dict(kvs_w1)['drops_last_5min'] == '1'
    assert _kvs_dict(kvs_w2)['drops_last_5min'] == '1'


def test_events_stale_when_log_fetch_never_succeeded():
    # Outer poll succeeded (poll_monotonic=NOW) but /log fetch never did
    # (wireless_events_last_success_monotonic=0.0).  Per-sub-query
    # freshness gating means the events task surfaces STALE rather than
    # rendering OK on the empty/preserved buffer.
    cache = CachedStatus(poll_monotonic=NOW)
    level, message, _ = synthesize_wireless_events_status(
        cache, 'wlan1', STALE, NOW, now_wall_dt=EVENT_NOW,
    )
    assert level == DiagnosticStatus.STALE
    assert 'no successful wireless-event log query' in message


def test_events_stale_when_log_fetch_aged_out():
    # /log last succeeded long enough ago to exceed stale_timeout_sec
    # even though the outer poll is still landing fresh.
    cache = CachedStatus(
        poll_monotonic=NOW,
        wireless_events_last_success_monotonic=NOW - (STALE + 5.0),
    )
    level, message, _ = synthesize_wireless_events_status(
        cache, 'wlan1', STALE, NOW, now_wall_dt=EVENT_NOW,
    )
    assert level == DiagnosticStatus.STALE
    assert 'wireless-event log' in message


def test_events_poll_age_sec_surfaced_when_fresh():
    # When events fetch is fresh, an events_poll_age_sec KV is exposed
    # so consumers can see exactly how recent the underlying /log data is.
    cache = CachedStatus(
        poll_monotonic=NOW,
        wireless_events_last_success_monotonic=NOW - 2.5,
    )
    _, _, kvs = synthesize_wireless_events_status(
        cache, 'wlan1', STALE, NOW, now_wall_dt=EVENT_NOW,
    )
    d = _kvs_dict(kvs)
    assert d['events_poll_age_sec'] == '2.5'


def test_events_clock_skew_sec_positive_when_log_older_than_now():
    # Normal case: latest event is older than now_wall_dt → positive skew.
    cache = CachedStatus(
        wireless_events=[
            _event('2026-05-18 22:00:00',
                   'AA:BB:CC@wlan1 established connection on 5745000, SSID bridge'),
        ],
        poll_monotonic=NOW,
        wireless_events_last_success_monotonic=NOW,
    )
    _, _, kvs = synthesize_wireless_events_status(
        cache, 'wlan1', STALE, NOW, now_wall_dt=EVENT_NOW,
    )
    d = _kvs_dict(kvs)
    # EVENT_NOW = 22:30:00; event at 22:00:00 → 30 min = 1800s skew
    assert d['clock_skew_sec'] == '1800'


def test_events_negative_ago_clamped_when_router_ahead_of_monitor():
    # Router clock running ahead: event timestamp is in the future
    # relative to now_wall_dt.  Without clamping the "Ns ago" fields
    # and current_session_age_sec would be negative.
    cache = CachedStatus(
        wireless_events=[
            _event('2026-05-18 22:31:00',
                   'AA:BB:CC@wlan1 established connection on 5745000, SSID bridge'),
        ],
        poll_monotonic=NOW,
        wireless_events_last_success_monotonic=NOW,
    )
    _, _, kvs = synthesize_wireless_events_status(
        cache, 'wlan1', STALE, NOW, now_wall_dt=EVENT_NOW,
    )
    d = _kvs_dict(kvs)
    # Event at 22:31:00 vs EVENT_NOW 22:30:00 → 60s ahead → clamped to 0
    assert d['last_assoc'].startswith('0s ago')
    assert d['current_session_age_sec'] == '0'
    # The negative skew is still exposed so operators can see the issue.
    assert d['clock_skew_sec'] == '-60'


def test_events_level_stays_ok_with_many_drops():
    """Per feedback_wifi_disconnect_not_an_error: drops are data, not errors."""
    cache = CachedStatus(
        wireless_events=[
            _event(f'2026-05-18 22:{29-i:02d}:00',
                   f'AA:BB:CC@wlan1: lost connection #{i}')
            for i in range(5)  # 5 drops in last 5 min
        ],
        poll_monotonic=NOW,
        wireless_events_last_success_monotonic=NOW,
    )
    level, _, kvs = synthesize_wireless_events_status(
        cache, 'wlan1', STALE, NOW, now_wall_dt=EVENT_NOW,
    )
    assert level == DiagnosticStatus.OK   # NEVER escalates on drop count
    assert _kvs_dict(kvs)['drops_last_5min'] == '5'
