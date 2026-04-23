# Copyright 2024 Roland Arsenault
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""Unit tests for network_tools.diagnostics_logic."""

from diagnostic_msgs.msg import DiagnosticStatus

from network_tools.diagnostics_logic import (
    PingSample,
    synthesize_ping_status,
)


STALE_TIMEOUT = 30.0
NOW = 100.0


def _kvs_dict(kvs):
    return {kv.key: kv.value for kv in kvs}


def test_never_polled_is_stale():
    """Before the first poll completes, status is STALE, not ERROR."""
    sample = PingSample(address='10.0.0.1')  # poll_monotonic = 0.0
    level, message, kvs = synthesize_ping_status(sample, STALE_TIMEOUT, NOW)

    assert level == DiagnosticStatus.STALE
    assert 'no successful poll yet' in message
    assert _kvs_dict(kvs)['reachable'] == 'False'


def test_cached_sample_aged_past_stale_timeout():
    """A sample older than stale_timeout_sec becomes STALE regardless of success."""
    sample = PingSample(
        address='10.0.0.1',
        success=True,
        latency_ms=1.0,
        loss_pct=0.0,
        poll_monotonic=NOW - STALE_TIMEOUT - 1.0,
        poll_wall_iso='2026-04-23T20:00:00+00:00',
    )
    level, message, _ = synthesize_ping_status(sample, STALE_TIMEOUT, NOW)

    assert level == DiagnosticStatus.STALE
    assert 'cached sample' in message
    assert 'stale_timeout_sec=30.0' in message


def test_ok_when_reachable_no_loss():
    sample = PingSample(
        address='10.0.0.1',
        success=True,
        latency_ms=12.3,
        loss_pct=0.0,
        ping_count=3,
        poll_monotonic=NOW - 1.0,
        poll_wall_iso='2026-04-23T20:00:00+00:00',
    )
    level, message, kvs = synthesize_ping_status(sample, STALE_TIMEOUT, NOW)

    assert level == DiagnosticStatus.OK
    assert '12.3 ms' in message
    d = _kvs_dict(kvs)
    assert d['latency_ms'] == '12.300'
    assert d['reachable'] == 'True'
    assert d['packet_loss_pct'] == '0.0'


def test_warn_when_partial_loss():
    sample = PingSample(
        address='10.0.0.1',
        success=True,
        latency_ms=5.0,
        loss_pct=33.0,
        ping_count=3,
        poll_monotonic=NOW,
        poll_wall_iso='2026-04-23T20:00:00+00:00',
    )
    level, message, _ = synthesize_ping_status(sample, STALE_TIMEOUT, NOW)

    assert level == DiagnosticStatus.WARN
    assert '33% packet loss' in message


def test_error_when_unreachable():
    sample = PingSample(
        address='10.0.0.1',
        success=False,
        latency_ms=0.0,
        loss_pct=100.0,
        ping_count=3,
        poll_monotonic=NOW,
    )
    level, message, _ = synthesize_ping_status(sample, STALE_TIMEOUT, NOW)

    assert level == DiagnosticStatus.ERROR
    assert message == 'Unreachable'


def test_error_uses_sample_error_message_if_set():
    """Custom error_message overrides the default 'Unreachable'."""
    sample = PingSample(
        address='10.0.0.1',
        success=False,
        error_message='ping binary not found',
        poll_monotonic=NOW,
    )
    level, message, _ = synthesize_ping_status(sample, STALE_TIMEOUT, NOW)

    assert level == DiagnosticStatus.ERROR
    assert message == 'ping binary not found'


def test_schema_drift_regression_no_connection_state_renaming():
    """
    Regression for ros2_network_monitor#15.

    The pre-Updater ping_monitor was NOT vulnerable to schema drift
    (ping_monitor always emitted one status per static target).
    This test locks in the invariant: the kvs set for a given target is
    the same shape whether the ping succeeds or fails, so downstream
    aggregators never see an orphan name register.
    """
    ok_sample = PingSample(
        address='10.0.0.1',
        success=True,
        latency_ms=1.0,
        loss_pct=0.0,
        ping_count=3,
        poll_monotonic=NOW,
    )
    err_sample = PingSample(
        address='10.0.0.1',
        success=False,
        loss_pct=100.0,
        ping_count=3,
        poll_monotonic=NOW,
    )

    _, _, ok_kvs = synthesize_ping_status(ok_sample, STALE_TIMEOUT, NOW)
    _, _, err_kvs = synthesize_ping_status(err_sample, STALE_TIMEOUT, NOW)

    # Same KeyValue keys on both paths — the shape of the status is
    # invariant to the connection result.
    assert sorted(_kvs_dict(ok_kvs).keys()) == sorted(_kvs_dict(err_kvs).keys())
