# Copyright 2024 Roland Arsenault
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""
ROS 2 node that pings a list of targets and publishes diagnostics.

Uses ``diagnostic_updater.Updater`` so each ping target gets a fixed-name
task that publishes at a steady cadence regardless of how long the
subprocess ping calls take.  Poll callback only updates per-target
``PingSample`` entries in a cache; task callbacks read the cache and
synthesize ``DiagnosticStatus`` via :mod:`network_tools.diagnostics_logic`.
"""

from datetime import datetime, timezone
import math
import subprocess
import threading
import time
from typing import Dict

import diagnostic_updater
from network_tools.diagnostics_logic import (
    PingSample,
    synthesize_ping_status,
)
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class PingMonitorNode(Node):

    def __init__(self):
        super().__init__('ping_monitor')

        self.declare_parameter('targets', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('poll_interval', 10.0)
        self.declare_parameter('update_period_sec', 1.0)
        self.declare_parameter('stale_timeout_sec', 0.0)
        self.declare_parameter('ping_count', 3)
        self.declare_parameter('ping_timeout', 5.0)
        self.declare_parameter('hardware_id', '')

        raw_targets = self.get_parameter('targets').value
        if not raw_targets:
            self.get_logger().error(
                'Parameter "targets" is required. '
                'Format: ["name:address", ...]'
            )
            raise SystemExit(1)

        self.targets: list[tuple[str, str]] = []
        for entry in raw_targets:
            if ':' not in entry:
                self.get_logger().error(
                    f'Invalid target format "{entry}". '
                    'Expected "name:address".'
                )
                raise SystemExit(1)
            name, address = entry.split(':', 1)
            name, address = name.strip(), address.strip()
            if not name or not address:
                self.get_logger().error(
                    f'Invalid target "{entry}": '
                    'name and address must be non-empty.'
                )
                raise SystemExit(1)
            self.targets.append((name, address))

        self.poll_interval = self.get_parameter('poll_interval').value
        update_period_sec = self.get_parameter('update_period_sec').value
        self.ping_count = self.get_parameter('ping_count').value
        self.ping_timeout = self.get_parameter('ping_timeout').value
        self.hardware_id = self.get_parameter('hardware_id').value

        # stale_timeout_sec = 0 means "auto": cached sample is considered
        # stale after 3 poll_intervals.  The 3× factor (vs. 2× for
        # mikrotik/teltonika) reflects that a sequential ping pass over
        # many targets with generous ping_timeout can legitimately take
        # ~2 poll_intervals to complete; tripling gives one missed poll
        # of headroom before the task flips to STALE.  Explicit values
        # override for field tuning.
        stale_timeout_sec = self.get_parameter('stale_timeout_sec').value
        if stale_timeout_sec <= 0.0:
            stale_timeout_sec = 3.0 * self.poll_interval
        self._stale_timeout_sec = stale_timeout_sec

        self._name_prefix = (
            f'Ping: {self.hardware_id}' if self.hardware_id else 'Ping'
        )

        # Per-target cache.  Replaced whole-dataclass under the lock so
        # task callbacks on the rclpy thread never see torn values.
        # Seeded with a placeholder so Updater tasks added at __init__
        # have something to read before the first poll completes.
        self._cache_lock = threading.Lock()
        self._cache: Dict[str, PingSample] = {
            name: PingSample(address=address)
            for name, address in self.targets
        }

        # diagnostic_updater.Updater publishes all registered tasks at
        # update_period_sec regardless of poll timing.  Tasks have fixed
        # names (one per static target) — no dynamic membership here.
        self._updater = diagnostic_updater.Updater(
            self, period=update_period_sec,
        )
        self._updater.setHardwareID(self.hardware_id or '')
        for name, _address in self.targets:
            task_name = f'{self._name_prefix}: {name}'
            self._updater.add(task_name, self._make_task(name))

        # Separate callback group so the Updater's publish timer can run
        # while a slow poll is in progress.  With per-target ping
        # deadlines of ping_count * ping_timeout, a single poll can block
        # for many seconds; without the split the update_period_sec
        # guarantee would be broken exactly when downstream consumers
        # most need fresh data.
        self.poll_timer = self.create_timer(
            self.poll_interval,
            self._poll_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        target_names = ', '.join(n for n, _ in self.targets)
        self.get_logger().info(
            f'Ping monitor started: [{target_names}] '
            f'poll every {self.poll_interval}s, '
            f'publish every {update_period_sec}s'
        )

    def _make_task(self, target_name: str):
        """Build an Updater task callback for one target."""
        def _task(stat):
            with self._cache_lock:
                sample = self._cache[target_name]
            level, message, kvs = synthesize_ping_status(
                sample,
                self._stale_timeout_sec,
                time.monotonic(),
            )
            stat.summary(level, message)
            for kv in kvs:
                stat.add(kv.key, kv.value)
            return stat
        return _task

    def _ping(self, address):
        """
        Ping an address and return (success, latency_ms, loss_pct, error).

        ``error`` is ``None`` on success, or a short human-readable string
        describing the failure mode (missing ping binary, subprocess
        timeout, nonzero exit).  Callers propagate it into
        ``PingSample.error_message`` so the ``: <target>`` task surfaces
        a specific reason instead of a generic "Unreachable".

        Uses the system ping command to avoid requiring raw socket
        privileges.
        """
        timeout_s = max(1, math.ceil(self.ping_timeout))
        deadline = timeout_s * self.ping_count + 1
        try:
            result = subprocess.run(
                [
                    'ping',
                    '-c', str(self.ping_count),
                    '-W', str(timeout_s),
                    '-w', str(deadline),
                    address,
                ],
                capture_output=True,
                text=True,
                timeout=deadline + 5,
            )
        except subprocess.TimeoutExpired:
            return False, 0.0, 100.0, (
                f'subprocess timeout after {deadline + 5}s'
            )
        except FileNotFoundError:
            return False, 0.0, 100.0, 'ping binary not found'

        loss = 100.0
        latency = 0.0

        for line in result.stdout.splitlines():
            if '% packet loss' in line:
                # "3 packets transmitted, 3 received, 0% packet loss"
                try:
                    loss_str = line.split('%')[0].rsplit(None, 1)[-1]
                    loss_str = loss_str.lstrip(',').strip()
                    loss = float(loss_str)
                except (ValueError, IndexError):
                    pass
            if line.startswith('rtt ') or line.startswith('round-trip '):
                # "rtt min/avg/max/mdev = 1.234/2.345/3.456/0.567 ms"
                try:
                    stats = line.split('=')[1].strip().split('/')[1]
                    latency = float(stats)
                except (ValueError, IndexError):
                    pass

        # Reachability is defined by packet loss, not ping's return code.
        # Linux ping exits 0 on full success, 1 on partial loss (host
        # IS reachable — should render as WARN), and 2 on network or
        # command errors (which always also produce 100% loss in the
        # parsed output, or the parser's default 100.0 if no summary
        # line appeared).  An earlier version used
        # ``returncode == 0 and loss < 100.0``, which wrongly collapsed
        # the partial-loss WARN case into ERROR via returncode==1.
        success = loss < 100.0
        error = None if success else 'Unreachable (100% packet loss)'
        return success, latency, loss, error

    def _poll_callback(self):
        """Ping every target sequentially; update cache atomically per target."""
        for name, address in self.targets:
            # Stamp each target individually — pings are sequential and
            # one target's ping_count * ping_timeout window can stretch
            # many seconds past the start of the loop.
            poll_wall_iso = datetime.now(timezone.utc).isoformat()
            success, latency_ms, loss_pct, error_message = self._ping(address)
            poll_monotonic = time.monotonic()

            new_sample = PingSample(
                address=address,
                success=success,
                latency_ms=latency_ms,
                loss_pct=loss_pct,
                ping_count=self.ping_count,
                poll_monotonic=poll_monotonic,
                poll_wall_iso=poll_wall_iso,
                error_message=error_message,
            )
            with self._cache_lock:
                self._cache[name] = new_sample


def main(args=None):
    rclpy.init(args=args)
    node = PingMonitorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
