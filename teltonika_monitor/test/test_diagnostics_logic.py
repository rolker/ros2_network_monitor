# Copyright 2024 Roland Arsenault
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""Unit tests for teltonika_monitor.diagnostics_logic."""

from diagnostic_msgs.msg import DiagnosticStatus

from teltonika_monitor.diagnostics_logic import (
    CachedStatus,
    cellular_quality,
    diff_dynamic_membership,
    interface_task_name,
    mwan3_task_name,
    parse_db,
    parse_dbm,
    synthesize_cellular_status,
    synthesize_connection_status,
    synthesize_interface_status,
    synthesize_mwan3_status,
    synthesize_system_status,
)


STALE = 10.0
NOW = 100.0
PREFIX = 'Teltonika: router.bizzy'


def _kvs_dict(kvs):
    return {kv.key: kv.value for kv in kvs}


# --- Parsing / quality helpers ---

def test_parse_dbm_and_db_variants():
    assert parse_dbm('-85 dBm') == -85.0
    assert parse_dbm('-85') == -85.0
    assert parse_dbm(None) is None
    assert parse_db('12.5 dB') == 12.5
    assert parse_db('bogus') is None


def test_cellular_quality_bands():
    assert cellular_quality(None, None)[0] == DiagnosticStatus.WARN
    assert cellular_quality(-70.0, 25.0)[0] == DiagnosticStatus.OK
    assert cellular_quality(-85.0, 15.0)[0] == DiagnosticStatus.OK
    assert cellular_quality(-95.0, 5.0)[0] == DiagnosticStatus.WARN
    assert cellular_quality(-115.0, -5.0)[0] == DiagnosticStatus.ERROR


def test_cellular_quality_negative_sinr_downgrades():
    # -85 is normally 4 bars (OK); negative SINR → 3 bars (WARN).
    level_neg, _ = cellular_quality(-85.0, -2.0)
    level_pos, _ = cellular_quality(-85.0, 10.0)
    assert level_neg == DiagnosticStatus.WARN
    assert level_pos == DiagnosticStatus.OK


# --- Connection task ---

def test_connection_stale_before_first_poll():
    level, message, _ = synthesize_connection_status(
        CachedStatus(), STALE, NOW,
    )
    assert level == DiagnosticStatus.STALE
    assert 'no successful poll' in message


def test_connection_ok():
    cache = CachedStatus(
        system_board={'model': 'RUTX11'},
        poll_monotonic=NOW - 1.0,
        poll_wall_iso='2026-04-23T20:00:00+00:00',
    )
    level, message, _ = synthesize_connection_status(cache, STALE, NOW)
    assert level == DiagnosticStatus.OK
    assert message == 'reachable'


def test_connection_error():
    cache = CachedStatus(
        poll_monotonic=NOW,
        error_message='Connection error: auth',
    )
    level, message, _ = synthesize_connection_status(cache, STALE, NOW)
    assert level == DiagnosticStatus.ERROR
    assert message == 'Connection error: auth'


def test_connection_stale_from_aged_cache():
    """Cache older than stale_timeout AND no error → STALE (polls stopped)."""
    cache = CachedStatus(
        system_board={'model': 'RUTX11'},
        poll_monotonic=NOW - (STALE + 5.0),
    )
    level, _, _ = synthesize_connection_status(cache, STALE, NOW)
    assert level == DiagnosticStatus.STALE


def test_connection_reports_error_on_first_poll_failure():
    """
    Regression for V5 (PR #19 round 2 review).

    After the cache-preservation fix, the first-poll-failure cache
    keeps the initial ``poll_monotonic=0.0`` default and sets
    ``error_message``.  The connection task must emit ERROR with the
    real reason — not the generic STALE "no successful poll yet" that
    an earlier version of ``synthesize_connection_status`` would have
    returned by checking staleness first.
    """
    first_fail_cache = CachedStatus(
        poll_monotonic=0.0,
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
    When a preserved cache ages past stale_timeout AND the most recent
    poll attempt errored, the error message is the freshest real news.
    """
    aged_error_cache = CachedStatus(
        system_board={'model': 'RUTX11'},
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

def test_system_status_includes_release_fields():
    cache = CachedStatus(
        system_board={
            'model': 'RUTX11',
            'hostname': 'router.bizzy',
            'release': {
                'distribution': 'Teltonika',
                'version': 'RUT9_R_00.07.05',
            },
        },
        poll_monotonic=NOW,
    )
    level, message, kvs = synthesize_system_status(cache, STALE, NOW)
    d = _kvs_dict(kvs)
    assert level == DiagnosticStatus.OK
    assert message == 'RUTX11'
    assert d['release.distribution'] == 'Teltonika'
    assert d['release.version'] == 'RUT9_R_00.07.05'


# --- Cellular task ---

def test_cellular_unsupported_warn():
    """Router with no modem — ubus reported method-not-found."""
    cache = CachedStatus(
        poll_monotonic=NOW,
        cellular_supported=False,
    )
    level, message, _ = synthesize_cellular_status(cache, STALE, NOW)
    assert level == DiagnosticStatus.WARN
    assert message == 'cellular not supported'


def test_cellular_no_service():
    cache = CachedStatus(
        poll_monotonic=NOW,
        cellular_signal={'net_mode': 'No service'},
    )
    level, message, _ = synthesize_cellular_status(cache, STALE, NOW)
    assert level == DiagnosticStatus.WARN
    assert message == 'No service'


def test_cellular_active_ok():
    cache = CachedStatus(
        poll_monotonic=NOW,
        cellular_signal={
            'net_mode': 'LTE',
            'rsrp': '-85 dBm',
            'sinr': '12 dB',
        },
    )
    level, message, kvs = synthesize_cellular_status(cache, STALE, NOW)
    assert level == DiagnosticStatus.OK
    assert 'LTE' in message
    assert '-85dBm' in message
    d = _kvs_dict(kvs)
    assert d['net_mode'] == 'LTE'
    assert d['rsrp'] == '-85 dBm'


# --- mwan3 task ---

def test_mwan3_online_ok():
    cache = CachedStatus(
        poll_monotonic=NOW,
        mwan3={'interfaces': {'wan1': {'status': 'online', 'score': 100}}},
    )
    level, message, kvs = synthesize_mwan3_status(cache, 'wan1', STALE, NOW)
    assert level == DiagnosticStatus.OK
    assert message == 'Online'
    assert _kvs_dict(kvs)['score'] == '100'


def test_mwan3_standby_ok_not_warn():
    """Standby is the expected steady state for a backup interface — not WARN."""
    cache = CachedStatus(
        poll_monotonic=NOW,
        mwan3={'interfaces': {'backup': {'status': 'standby'}}},
    )
    level, message, _ = synthesize_mwan3_status(cache, 'backup', STALE, NOW)
    assert level == DiagnosticStatus.OK
    assert message == 'Standby'


def test_mwan3_offline_error():
    cache = CachedStatus(
        poll_monotonic=NOW,
        mwan3={'interfaces': {'wan1': {'status': 'offline'}}},
    )
    level, message, _ = synthesize_mwan3_status(cache, 'wan1', STALE, NOW)
    assert level == DiagnosticStatus.ERROR
    assert message == 'Offline'


def test_mwan3_missing_member_stale():
    cache = CachedStatus(
        poll_monotonic=NOW,
        mwan3={'interfaces': {'wan1': {'status': 'online'}}},
    )
    level, message, _ = synthesize_mwan3_status(
        cache, 'wan-gone', STALE, NOW,
    )
    assert level == DiagnosticStatus.STALE
    assert 'wan-gone' in message


def test_mwan3_unsupported_stale():
    cache = CachedStatus(poll_monotonic=NOW, mwan3_supported=False)
    level, message, _ = synthesize_mwan3_status(cache, 'wan1', STALE, NOW)
    assert level == DiagnosticStatus.STALE
    assert 'not supported' in message


def test_mwan3_track_ip_rendered():
    cache = CachedStatus(
        poll_monotonic=NOW,
        mwan3={'interfaces': {'wan1': {
            'status': 'online',
            'track_ip': [
                {'ip': '8.8.8.8', 'status': 'online', 'latency': 10, 'packetloss': 0},
            ],
        }}},
    )
    _, _, kvs = synthesize_mwan3_status(cache, 'wan1', STALE, NOW)
    d = _kvs_dict(kvs)
    assert d['track/8.8.8.8'] == 'online latency=10 loss=0'


# --- Network interface task ---

def test_interface_up_ok():
    cache = CachedStatus(
        poll_monotonic=NOW,
        network_interfaces=[
            {'interface': 'lan', 'up': True, 'proto': 'static'},
        ],
    )
    level, message, _ = synthesize_interface_status(
        cache, 'lan', STALE, NOW,
    )
    assert level == DiagnosticStatus.OK
    assert message == 'Up'


def test_interface_down_warn():
    cache = CachedStatus(
        poll_monotonic=NOW,
        network_interfaces=[
            {'interface': 'wwan', 'up': False},
        ],
    )
    level, message, _ = synthesize_interface_status(
        cache, 'wwan', STALE, NOW,
    )
    assert level == DiagnosticStatus.WARN
    assert message == 'Down'


def test_interface_ipv4_addresses_rendered():
    cache = CachedStatus(
        poll_monotonic=NOW,
        network_interfaces=[{
            'interface': 'lan',
            'up': True,
            'ipv4-address': [{'address': '192.168.20.1', 'mask': 24}],
        }],
    )
    _, _, kvs = synthesize_interface_status(cache, 'lan', STALE, NOW)
    assert _kvs_dict(kvs)['ipv4'] == '192.168.20.1/24'


# --- Cache preservation on transient failure (F2/V1 regression) ---

def test_cache_preserved_across_transient_failure():
    """
    Regression for F2/V1 (PR #19).

    After the preservation fix, a cache representing "last successful
    poll was 1s ago, most recent poll attempt failed" must let non-
    connection tasks continue to render last-known values.  Without
    the fix, system_board would be None and every task would
    immediately flip to ERROR/STALE on a single transient timeout.
    """
    preserved_cache = CachedStatus(
        system_board={'model': 'RUTX11'},
        cellular_signal={'net_mode': 'LTE', 'rsrp': '-85 dBm', 'sinr': '10 dB'},
        cellular_supported=True,
        mwan3={'interfaces': {'wan1': {'status': 'online'}}},
        mwan3_supported=True,
        network_interfaces=[{'interface': 'lan', 'up': True}],
        poll_monotonic=NOW - 1.0,  # previous successful poll, still fresh
        poll_wall_iso='2026-04-23T20:00:00+00:00',
        error_message='Connection error: auth',  # most recent attempt
    )

    # : connection surfaces the error.
    level, message, _ = synthesize_connection_status(
        preserved_cache, STALE, NOW,
    )
    assert level == DiagnosticStatus.ERROR
    assert 'auth' in message

    # Other fixed + dynamic tasks render from preserved data.
    level, message, _ = synthesize_system_status(preserved_cache, STALE, NOW)
    assert level == DiagnosticStatus.OK
    assert message == 'RUTX11'

    level, message, _ = synthesize_cellular_status(preserved_cache, STALE, NOW)
    assert level == DiagnosticStatus.OK
    assert 'LTE' in message

    level, message, _ = synthesize_mwan3_status(
        preserved_cache, 'wan1', STALE, NOW,
    )
    assert level == DiagnosticStatus.OK
    assert message == 'Online'

    level, message, _ = synthesize_interface_status(
        preserved_cache, 'lan', STALE, NOW,
    )
    assert level == DiagnosticStatus.OK
    assert message == 'Up'


def test_preserved_cache_ages_into_stale():
    """Once poll_monotonic ages past stale_timeout_sec, tasks flip to STALE."""
    cache = CachedStatus(
        system_board={'model': 'RUTX11'},
        cellular_signal={'net_mode': 'LTE', 'rsrp': '-85 dBm'},
        mwan3={'interfaces': {'wan1': {'status': 'online'}}},
        network_interfaces=[{'interface': 'lan', 'up': True}],
        poll_monotonic=NOW - (STALE + 5.0),
        poll_wall_iso='2026-04-23T19:00:00+00:00',
        error_message='Connection error: auth',
    )
    for name, fn in [
        ('system', synthesize_system_status),
        ('cellular', synthesize_cellular_status),
    ]:
        level, _, _ = fn(cache, STALE, NOW)
        assert level == DiagnosticStatus.STALE, f'{name} should be STALE'

    level, _, _ = synthesize_mwan3_status(cache, 'wan1', STALE, NOW)
    assert level == DiagnosticStatus.STALE
    level, _, _ = synthesize_interface_status(cache, 'lan', STALE, NOW)
    assert level == DiagnosticStatus.STALE


# --- Schema-drift regression ---

def test_schema_drift_connection_task_stable_across_states():
    """
    Regression for ros2_network_monitor#15.

    The pre-Updater node synthesized a "Teltonika: <hwid>" bare-summary
    status on connection error, a different name from the happy-path
    ": system" / ": cellular" / etc. tasks.  With Updater the task set
    is fixed at .add() time; the error path can only modify task
    CONTENT, not task NAMES.  This test locks in that invariant.
    """
    ok_cache = CachedStatus(
        system_board={'model': 'RUTX11'},
        poll_monotonic=NOW,
        error_message=None,
    )
    err_cache = CachedStatus(
        poll_monotonic=NOW,
        error_message='Connection error: refused',
    )

    ok_level, _, _ = synthesize_connection_status(ok_cache, STALE, NOW)
    err_level, _, _ = synthesize_connection_status(err_cache, STALE, NOW)
    assert ok_level == DiagnosticStatus.OK
    assert err_level == DiagnosticStatus.ERROR

    # A per-mwan3 task registered before the error still exists; its
    # callback reads the now-error-flagged cache and renders STALE,
    # NOT a differently-named status.
    err_mwan_level, err_mwan_msg, _ = synthesize_mwan3_status(
        err_cache, 'wan1', STALE, NOW,
    )
    assert err_mwan_level == DiagnosticStatus.STALE
    assert 'wan1' in err_mwan_msg


# --- Dynamic-membership diff ---

def test_diff_add_remove_and_grace():
    # First poll: a and b observed.
    observed = {'mwan3/a', 'mwan3/b'}
    to_add, to_remove, last_seen = diff_dynamic_membership(
        observed, set(), {}, grace_sec=5.0, now_monotonic=NOW,
    )
    assert to_add == observed
    assert to_remove == set()
    assert last_seen == {'mwan3/a': NOW, 'mwan3/b': NOW}

    # Next poll (NOW + 3): only a observed. b within grace → kept.
    observed = {'mwan3/a'}
    to_add, to_remove, last_seen = diff_dynamic_membership(
        observed, set(last_seen.keys()), last_seen,
        grace_sec=5.0, now_monotonic=NOW + 3.0,
    )
    assert to_add == set()
    assert to_remove == set()
    assert last_seen['mwan3/a'] == NOW + 3.0
    assert last_seen['mwan3/b'] == NOW  # retained

    # Next poll (NOW + 10): still only a. b past grace → remove.
    observed = {'mwan3/a'}
    to_add, to_remove, last_seen = diff_dynamic_membership(
        observed, set(last_seen.keys()), last_seen,
        grace_sec=5.0, now_monotonic=NOW + 10.0,
    )
    assert to_add == set()
    assert to_remove == {'mwan3/b'}
    assert 'mwan3/b' not in last_seen


# --- Task-name helpers ---

def test_task_name_helpers():
    assert mwan3_task_name(PREFIX, 'mob1s1a1') == (
        'Teltonika: router.bizzy: mwan3/mob1s1a1'
    )
    assert interface_task_name(PREFIX, 'wan') == (
        'Teltonika: router.bizzy: interface/wan'
    )
