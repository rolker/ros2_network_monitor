"""ROS 2 node that polls a MikroTik device and publishes diagnostics."""

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

        self.timer = self.create_timer(poll_interval, self.poll_callback)
        self.get_logger().info(
            f'Monitoring MikroTik device at {host}:{port} '
            f'every {poll_interval}s'
        )

    def poll_callback(self):
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        try:
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
            self.get_logger().warn(f'Failed to poll device: {e}')

        self.diag_pub.publish(msg)

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
        except RouterOSClientError:
            # Device may not have wireless interfaces — not an error
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
            status.level = DiagnosticStatus.OK
            status.message = 'Associated'

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
