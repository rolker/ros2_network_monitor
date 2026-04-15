# Copyright 2024 Roland Arsenault
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""ROS 2 node that pings a list of targets and publishes diagnostics."""

import math
import subprocess

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.node import Node


class PingMonitorNode(Node):

    def __init__(self):
        super().__init__('ping_monitor')

        self.declare_parameter('targets', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('poll_interval', 10.0)
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
        self.ping_count = self.get_parameter(
            'ping_count'
        ).value
        self.ping_timeout = self.get_parameter(
            'ping_timeout'
        ).value
        self.hw_id = self.get_parameter('hardware_id').value
        self._name_prefix = (
            f'Ping: {self.hw_id}' if self.hw_id else 'Ping'
        )

        self.diag_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10
        )

        self.timer = self.create_timer(
            self.poll_interval, self.poll_callback
        )

        target_names = ', '.join(n for n, _ in self.targets)
        self.get_logger().info(
            f'Ping monitor started: [{target_names}] '
            f'every {self.poll_interval}s'
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
        msg.header.stamp = self.get_clock().now().to_msg()

        for name, address in self.targets:
            success, latency_ms, loss_pct = self._ping(address)

            status = DiagnosticStatus()
            status.name = f'{self._name_prefix}: {name}'
            status.hardware_id = self.hw_id or address

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

            msg.status.append(status)

        self.diag_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PingMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
