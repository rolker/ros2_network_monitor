"""ROS 2 node that polls a MikroTik device and publishes diagnostics."""

import threading
from datetime import datetime, timezone

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from mikrotik_monitor.routeros_client import RouterOSClient, RouterOSClientError


class MikroTikMonitorNode(Node):

    def __init__(self):
        super().__init__('mikrotik_monitor')

        # Parameters
        self.declare_parameter('host', '')
        self.declare_parameter('username', 'admin')
        self.declare_parameter('password', '')
        self.declare_parameter('port', 80)
        self.declare_parameter('use_ssl', False)
        self.declare_parameter('poll_interval', 5.0)
        self.declare_parameter('publish_interval', 1.0)
        self.declare_parameter('max_data_age_s', 0.0)
        self.declare_parameter('hardware_id', '')
        self.declare_parameter('verify_ssl', False)
        self.declare_parameter('ignored_interfaces', [])

        host = self.get_parameter('host').get_parameter_value().string_value
        if not host:
            self.get_logger().error('Parameter "host" is required')
            raise SystemExit(1)

        username = self.get_parameter(
            'username'
        ).get_parameter_value().string_value
        password = self.get_parameter(
            'password'
        ).get_parameter_value().string_value
        port = self.get_parameter(
            'port'
        ).get_parameter_value().integer_value
        use_ssl = self.get_parameter(
            'use_ssl'
        ).get_parameter_value().bool_value
        poll_interval = self.get_parameter(
            'poll_interval'
        ).get_parameter_value().double_value
        publish_interval = self.get_parameter(
            'publish_interval'
        ).get_parameter_value().double_value
        self.hardware_id = self.get_parameter(
            'hardware_id'
        ).get_parameter_value().string_value
        verify_ssl = self.get_parameter(
            'verify_ssl'
        ).get_parameter_value().bool_value
        # Denylist of interface names whose diagnostics should not be
        # published (e.g. unused ports that are intentionally down).
        self._ignored_interfaces = set(
            self.get_parameter(
                'ignored_interfaces'
            ).get_parameter_value().string_array_value
        )

        self.client = RouterOSClient(
            host=host,
            username=username,
            password=password,
            port=port,
            use_ssl=use_ssl,
            verify_ssl=verify_ssl,
        )

        # Use host as hardware_id if not specified
        if not self.hardware_id:
            self.hardware_id = host

        self.diag_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10
        )

        # Cache of the most recent poll result (success or failure);
        # republished at publish_interval to avoid false STALE in
        # downstream consumers. On client error the cached message
        # carries the ERROR status so subscribers see the device is
        # down persistently between retries.
        self._cached_msg: DiagnosticArray | None = None
        self._last_poll_time = None
        self._cache_lock = threading.Lock()

        # Stale threshold: if no successful poll within max_data_age_s,
        # republished statuses are downgraded to STALE so consumers that
        # only read DiagnosticStatus.level (not the last_query_time
        # KeyValue) still see real polling stalls. 0 = auto, 3*poll.
        max_data_age_s = self.get_parameter(
            'max_data_age_s'
        ).get_parameter_value().double_value
        if max_data_age_s <= 0.0:
            max_data_age_s = 3.0 * poll_interval
        self._max_data_age = Duration(seconds=max_data_age_s)

        # Separate callback groups so the publish timer can run while a
        # slow poll is in progress — required by the MultiThreadedExecutor
        # in main(). RouterOSClient HTTP requests can stall for tens of
        # seconds when the device is unreachable; without the split the
        # publish_interval guarantee would be broken exactly when
        # downstream consumers most need fresh data.
        self.poll_timer = self.create_timer(
            poll_interval,
            self.poll_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.publish_timer = self.create_timer(
            publish_interval,
            self._publish_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.get_logger().info(
            f'Monitoring MikroTik device at {host}:{port} '
            f'poll every {poll_interval}s, '
            f'publish every {publish_interval}s'
        )

    def poll_callback(self):
        msg = DiagnosticArray()
        query_time_iso = datetime.now(timezone.utc).isoformat()

        try:
            self._add_system_diagnostics(msg)
            self._add_interface_diagnostics(msg)
            self._add_wireless_diagnostics(msg)
        except RouterOSClientError as e:
            # Publish error status so diagnostics shows the device is down
            status = DiagnosticStatus()
            status.level = DiagnosticStatus.ERROR
            status.name = f'MikroTik: {self.hardware_id}'
            status.hardware_id = self.hardware_id
            status.message = f'Connection error: {e}'
            msg.status.append(status)
            self.get_logger().warning(f'Failed to poll device: {e}')

        # Stamp every status with the actual query time so observers can
        # compute true data age across republishes.
        for status in msg.status:
            status.values.append(
                KeyValue(key='last_query_time', value=query_time_iso)
            )
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
        if last_poll is not None and (now - last_poll) > self._max_data_age:
            age_s = (now - last_poll).nanoseconds / 1e9
            for status in msg.status:
                status.level = DiagnosticStatus.STALE
                status.message = f'STALE: {age_s:.1f}s since last poll'

        self.diag_pub.publish(msg)

    def _add_system_diagnostics(self, msg: DiagnosticArray):
        resource = self.client.get_system_resource()
        status = DiagnosticStatus()
        status.name = f'MikroTik: {self.hardware_id}: system'
        status.hardware_id = self.hardware_id
        status.level = DiagnosticStatus.OK
        status.message = resource.get('board-name', 'unknown')

        fields = [
            'board-name', 'version', 'uptime',
            'cpu-load', 'cpu-count', 'cpu-frequency',
            'free-memory', 'total-memory',
            'free-hdd-space', 'total-hdd-space',
        ]
        for field in fields:
            if field in resource:
                status.values.append(
                    KeyValue(key=field, value=str(resource[field]))
                )

        # Add health data (temperature, voltage) if available
        try:
            health = self.client.get_system_health()
            if isinstance(health, list):
                for entry in health:
                    name = entry.get('name', '')
                    value = entry.get('value', '')
                    if name and value:
                        status.values.append(
                            KeyValue(key=name, value=str(value))
                        )
            elif isinstance(health, dict):
                for key, value in health.items():
                    if key != '.id':
                        status.values.append(
                            KeyValue(key=key, value=str(value))
                        )
        except RouterOSClientError:
            pass  # Not all devices support /system/health

        msg.status.append(status)

    def _add_interface_diagnostics(self, msg: DiagnosticArray):
        interfaces = self.client.get_interfaces()
        for iface in interfaces:
            iface_name = iface.get('name', 'unknown')
            if iface_name in self._ignored_interfaces:
                continue
            status = DiagnosticStatus()
            status.name = (
                f'MikroTik: {self.hardware_id}: '
                f'interface/{iface_name}'
            )
            status.hardware_id = self.hardware_id

            running = str(iface.get('running', 'false')).lower() == 'true'
            disabled = str(iface.get('disabled', 'false')).lower() == 'true'

            if disabled:
                status.level = DiagnosticStatus.WARN
                status.message = 'Disabled'
            elif not running:
                status.level = DiagnosticStatus.WARN
                status.message = 'Not running'
            else:
                status.level = DiagnosticStatus.OK
                status.message = 'Running'

            # Report available fields
            fields = [
                'type', 'mtu', 'running', 'disabled',
                'tx-byte', 'rx-byte',
                'tx-packet', 'rx-packet',
                'tx-error', 'rx-error',
                'tx-drop', 'rx-drop',
                'link-downs',
            ]
            for field in fields:
                if field in iface:
                    status.values.append(
                        KeyValue(key=field, value=str(iface[field]))
                    )

            msg.status.append(status)

    def _add_wireless_diagnostics(self, msg: DiagnosticArray):
        try:
            registrations = self.client.get_wireless_registrations()
        except RouterOSClientError as e:
            if e.http_code in (400, 404):
                # Device doesn't have wireless interfaces — not an error
                return
            self.get_logger().warning(
                f'Failed to query wireless registrations: {e}'
            )
            return

        for reg in registrations:
            mac = reg.get('mac-address', 'unknown')
            interface = reg.get('interface', 'unknown')
            if interface in self._ignored_interfaces:
                continue
            status = DiagnosticStatus()
            status.name = (
                f'MikroTik: {self.hardware_id}: '
                f'wireless/{interface}/{mac}'
            )
            status.hardware_id = self.hardware_id

            snr = self._parse_snr(reg.get('signal-to-noise'))
            level, bars = self._wireless_quality(snr)
            status.level = level
            snr_str = f' SNR {snr:.0f}dB' if snr is not None else ''
            status.message = f'Associated{snr_str} {bars}'

            fields = [
                'interface', 'mac-address',
                'signal-strength', 'signal-to-noise',
                'noise-floor',
                'tx-rate', 'rx-rate',
                'tx-ccq', 'rx-ccq',
                'uptime',
                'bytes', 'packets',
                'frames',
            ]
            for field in fields:
                if field in reg:
                    status.values.append(
                        KeyValue(key=field, value=str(reg[field]))
                    )

            msg.status.append(status)

    @staticmethod
    def _parse_snr(value):
        """Parse an SNR string like '22@HT40' or '22 dB' to float."""
        if value is None:
            return None
        try:
            return float(str(value).split('@')[0].split()[0])
        except (ValueError, IndexError):
            return None

    @staticmethod
    def _wireless_quality(snr):
        """Map SNR to a diagnostic level and signal bars string.

        Thresholds for 5 GHz point-to-point link:
          Excellent: SNR > 30
          Good:      SNR > 20
          Fair:      SNR > 15
          Poor:      SNR > 10
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


def main(args=None):
    rclpy.init(args=args)
    node = MikroTikMonitorNode()
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
