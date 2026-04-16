"""ROS 2 node that polls a MikroTik device and publishes diagnostics."""

from datetime import datetime, timezone

import rclpy
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
        self.declare_parameter('hardware_id', '')
        self.declare_parameter('verify_ssl', False)

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

        # Cache of the most recent successful poll; republished at
        # publish_interval to avoid false STALE in downstream consumers.
        self._cached_msg: DiagnosticArray | None = None

        self.poll_timer = self.create_timer(
            poll_interval, self.poll_callback
        )
        self.publish_timer = self.create_timer(
            publish_interval, self._publish_callback
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
        self._cached_msg = msg

    def _publish_callback(self):
        if self._cached_msg is None:
            return
        # Refresh send timestamp; per-status last_query_time is left intact.
        self._cached_msg.header.stamp = self.get_clock().now().to_msg()
        self.diag_pub.publish(self._cached_msg)

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
            status = DiagnosticStatus()
            status.name = (
                f'MikroTik: {self.hardware_id}: '
                f'interface/{iface.get("name", "unknown")}'
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
            status = DiagnosticStatus()
            mac = reg.get('mac-address', 'unknown')
            interface = reg.get('interface', 'unknown')
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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
