# Copyright 2024 Roland Arsenault
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""
Pure-Python diagnostic logic for the ping monitor.

Avoids ROS 2 runtime dependencies (rclpy) so it can be unit-tested
against hand-built samples.  Still imports DiagnosticStatus for the
standard level constants.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


@dataclass
class PingSample:
    """
    Latest ping result for one target, plus metadata.

    The poll callback replaces this whole-dataclass under a lock so task
    callbacks on the rclpy thread only ever see a consistent snapshot.
    ``poll_monotonic == 0.0`` means "no poll has completed yet."
    """

    address: str
    success: bool = False
    latency_ms: float = 0.0
    loss_pct: float = 100.0
    ping_count: int = 0
    poll_monotonic: float = 0.0
    poll_wall_iso: str = ''
    error_message: Optional[str] = None  # non-None if the ping invocation itself failed


def synthesize_ping_status(
    sample: PingSample,
    stale_timeout_sec: float,
    now_monotonic: float,
) -> tuple[int, str, list[KeyValue]]:
    """
    Render a (level, message, kvs) tuple from a cached ping sample.

    - STALE when no poll has completed yet (``poll_monotonic == 0.0``) or
      when the cached sample is older than ``stale_timeout_sec``.
    - ERROR when the last ping failed (unreachable or loss == 100%).
    - WARN when the last ping had any packet loss below 100%.
    - OK otherwise.

    The task name is supplied by the node; this function only renders
    the contents of one DiagnosticStatus.
    """
    kvs = _render_kvs(sample)

    if sample.poll_monotonic == 0.0:
        return (
            DiagnosticStatus.STALE,
            'no successful poll yet',
            kvs,
        )

    age = now_monotonic - sample.poll_monotonic
    if age > stale_timeout_sec:
        return (
            DiagnosticStatus.STALE,
            f'cached sample {age:.1f}s old '
            f'(stale_timeout_sec={stale_timeout_sec})',
            kvs,
        )

    if not sample.success:
        return (
            DiagnosticStatus.ERROR,
            sample.error_message or 'Unreachable',
            kvs,
        )

    if sample.loss_pct > 0.0:
        return (
            DiagnosticStatus.WARN,
            f'{sample.loss_pct:.0f}% packet loss',
            kvs,
        )

    return (
        DiagnosticStatus.OK,
        f'{sample.latency_ms:.1f} ms',
        kvs,
    )


def _render_kvs(sample: PingSample) -> list[KeyValue]:
    """Build the KeyValue payload for a ping sample."""
    return [
        KeyValue(key='address', value=sample.address),
        KeyValue(key='latency_ms', value=f'{sample.latency_ms:.3f}'),
        KeyValue(key='packet_loss_pct', value=f'{sample.loss_pct:.1f}'),
        KeyValue(key='reachable', value=str(sample.success)),
        KeyValue(key='ping_count', value=str(sample.ping_count)),
        KeyValue(
            key='last_query_time',
            value=sample.poll_wall_iso or 'never',
        ),
    ]
