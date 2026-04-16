# Copyright 2024 Roland Arsenault
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""ROS 2 node that pings a list of targets and publishes diagnostics."""

import math
import subprocess
import threading
from datetime import datetime, timezone

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class PingMonitorNode(Node):

    def __init__(self):
        super().__init__('ping_monitor')

        self.declare_parameter('targets', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('poll_interval', 10.0)
        self.declare_parameter('publish_interval', 1.0)
        self.declare_parameter('max_data_age_s', 0.0)
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

        self.targets = []
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

        self.poll_interval = self.get_parameter(
            'poll_interval'
        ).value
        publish_interval = self.get_parameter(
            'publish_interval'
        ).value
        self.ping_count = self.get_parameter(
            'ping_count'
        ).value
        self.ping_timeout = self.get_parameter(
            'ping_timeout'
        ).value
        self.hardware_id = self.get_parameter('hardware_id').value
        self._name_prefix = (
            f'Ping: {self.hardware_id}' if self.hardware_id else 'Ping'
        )

        self.diag_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10
        )

        # Cache of the most recent poll result (success or failure).
        # Republished at publish_interval so downstream consumers
        # (rqt_runtime_monitor's 5 s stale window, aggregator analyzers,
        # annunciator stale timeouts) don't see false STALE between
        # polls. Unreachable targets surface as ERROR statuses that get
        # cached and persistently republished until the next poll.
        self._cached_msg: DiagnosticArray | None = None
        self._last_poll_time = None
        self._cache_lock = threading.Lock()

        # Stale threshold: if no successful poll within max_data_age_s,
        # republished statuses are downgraded to STALE so consumers that
        # only read DiagnosticStatus.level (not the last_query_time
        # KeyValue) still see real polling stalls. 0 = auto, 3*poll.
        max_data_age_s = self.get_parameter(
            'max_data_age_s'
        ).value
        if max_data_age_s <= 0.0:
            max_data_age_s = 3.0 * self.poll_interval
        self._max_data_age = Duration(seconds=max_data_age_s)

        # Separate callback groups so the publish timer can run while a
        # slow poll is in progress — required by the MultiThreadedExecutor
        # in main(). With per-target ping deadlines of ping_count *
        # ping_timeout, a single poll can block for many seconds; without
        # the split the publish_interval guarantee would be broken
        # exactly when downstream consumers most need fresh data.
        self.poll_timer = self.create_timer(
            self.poll_interval,
            self.poll_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.publish_timer = self.create_timer(
            publish_interval,
            self._publish_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        target_names = ', '.join(n for n, _ in self.targets)
        self.get_logger().info(
            f'Ping monitor started: [{target_names}] '
            f'poll every {self.poll_interval}s, '
            f'publish every {publish_interval}s'
        )

    def _ping(self, address):
        """
        Ping an address and return (success, latency_ms, loss_pct).

        Uses the system ping command to avoid requiring raw socket privileges.
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
            return False, 0.0, 100.0
        except FileNotFoundError:
            return False, 0.0, 100.0

        loss = 100.0
        latency = 0.0

        for line in result.stdout.splitlines():
            if '% packet loss' in line:
                # "3 packets transmitted, 3 received, 0% packet loss"
                try:
                    loss_str = line.split('%')[0].rsplit(None, 1)[-1]
                    # Remove any leading comma
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

        success = result.returncode == 0 and loss < 100.0
        return success, latency, loss

    def poll_callback(self):
        msg = DiagnosticArray()

        for name, address in self.targets:
            # Stamp each target individually — pings are sequential and
            # individual ping_count * ping_timeout windows can stretch
            # the per-target loop across many seconds.
            query_time_iso = datetime.now(timezone.utc).isoformat()
            success, latency_ms, loss_pct = self._ping(address)

            status = DiagnosticStatus()
            status.name = f'{self._name_prefix}: {name}'
            status.hardware_id = self.hardware_id or address

            if not success:
                status.level = DiagnosticStatus.ERROR
                status.message = 'Unreachable'
            elif loss_pct > 0.0:
                status.level = DiagnosticStatus.WARN
                status.message = f'{loss_pct:.0f}% packet loss'
            else:
                status.level = DiagnosticStatus.OK
                status.message = f'{latency_ms:.1f} ms'

            status.values.append(
                KeyValue(key='address', value=address)
            )
            status.values.append(
                KeyValue(key='latency_ms', value=f'{latency_ms:.3f}')
            )
            status.values.append(
                KeyValue(key='packet_loss_pct', value=f'{loss_pct:.1f}')
            )
            status.values.append(
                KeyValue(key='reachable', value=str(success))
            )
            status.values.append(
                KeyValue(key='ping_count', value=str(self.ping_count))
            )
            status.values.append(
                KeyValue(key='last_query_time', value=query_time_iso)
            )

            msg.status.append(status)

        with self._cache_lock:
            self._cached_msg = msg
            self._last_poll_time = self.get_clock().now()

    def _publish_callback(self):
        with self._cache_lock:
            msg = self._cached_msg
            last_poll = self._last_poll_time
        if msg is None:
            return

        now = self.get_clock().now()
        msg.header.stamp = now.to_msg()

        # Downgrade levels to STALE if the cached data is older than
        # max_data_age. Consumers that only read status.level (not the
        # last_query_time KeyValue) need this to detect polling stalls.
        # Mutates the cached msg in place — safe because the next
        # successful poll fully replaces the cache.
        if last_poll is not None and (now - last_poll) > self._max_data_age:
            age_s = (now - last_poll).nanoseconds / 1e9
            for status in msg.status:
                status.level = DiagnosticStatus.STALE
                status.message = f'STALE: {age_s:.1f}s since last poll'

        self.diag_pub.publish(msg)


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
