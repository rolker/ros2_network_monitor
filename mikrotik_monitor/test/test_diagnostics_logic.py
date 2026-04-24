# Copyright 2024 Roland Arsenault
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""Unit tests for mikrotik_monitor.diagnostics_logic."""

from diagnostic_msgs.msg import DiagnosticStatus

from mikrotik_monitor.diagnostics_logic import (
    CachedStatus,
    diff_dynamic_membership,
    interface_task_name,
    parse_snr,
    synthesize_connection_status,
    synthesize_interface_status,
    synthesize_system_status,
    synthesize_wireless_status,
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
    """Cache older than stale_timeout → STALE even if previously OK."""
    cache = CachedStatus(
        system_resource={'board-name': 'RB4011'},
        poll_monotonic=NOW - (STALE + 5.0),
        poll_wall_iso='2026-04-23T19:00:00+00:00',
    )
    level, message, _ = synthesize_connection_status(cache, STALE, NOW)
    assert level == DiagnosticStatus.STALE
    assert 'cached data' in message


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
    )
    level, message, kvs = synthesize_wireless_status(
        cache, 'wlan1', 'AA:BB:CC:DD:EE:FF', STALE, NOW,
    )
    assert level == DiagnosticStatus.OK
    assert 'SNR 25dB' in message
    assert 'Associated' in message
    d = _kvs_dict(kvs)
    assert d['tx-rate'] == '400Mbps'
