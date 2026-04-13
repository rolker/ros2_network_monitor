"""ROS 2 node that polls a Teltonika router and publishes diagnostics."""

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from teltonika_monitor.ubus_client import UbusClient, UbusClientError


class TeltonikaMonitorNode(Node):

    def __init__(self):
        super().__init__('teltonika_monitor')

        # Parameters
        self.declare_parameter('host', '')
        self.declare_parameter('username', 'ros_monitor')
        self.declare_parameter('password', '')
        self.declare_parameter('port', 443)
        self.declare_parameter('use_ssl', True)
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

        self.client = UbusClient(
            host=host,
            username=username,
            password=password,
            port=port,
            use_ssl=use_ssl,
            verify_ssl=verify_ssl,
        )

        if not self.hardware_id:
            self.hardware_id = host

        self.diag_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10
        )

        self.timer = self.create_timer(poll_interval, self.poll_callback)
        self.get_logger().info(
            f'Monitoring Teltonika router at {host}:{port} '
            f'every {poll_interval}s'
        )

    def poll_callback(self):
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        try:
            self._add_system_diagnostics(msg)
            self._add_cellular_diagnostics(msg)
            self._add_mwan3_diagnostics(msg)
            self._add_interface_diagnostics(msg)
        except UbusClientError as e:
            status = DiagnosticStatus()
            status.level = DiagnosticStatus.ERROR
            status.name = f'Teltonika: {self.hardware_id}'
            status.hardware_id = self.hardware_id
            status.message = f'Connection error: {e}'
            msg.status.append(status)
            self.get_logger().warning(f'Failed to poll router: {e}')

        self.diag_pub.publish(msg)

    def _add_system_diagnostics(self, msg: DiagnosticArray):
        board = self.client.get_system_board()
        status = DiagnosticStatus()
        status.name = f'Teltonika: {self.hardware_id}: system'
        status.hardware_id = self.hardware_id
        status.level = DiagnosticStatus.OK
        status.message = board.get('model', 'unknown')

        for field in ['model', 'hostname', 'kernel', 'system']:
            if field in board:
                status.values.append(
                    KeyValue(key=field, value=str(board[field]))
                )

        release = board.get('release', {})
        for field in ['distribution', 'version', 'description']:
            if field in release:
                status.values.append(
                    KeyValue(key=f'release.{field}',
                             value=str(release[field]))
                )

        msg.status.append(status)

    def _add_cellular_diagnostics(self, msg: DiagnosticArray):
        try:
            signal = self.client.get_signal()
        except UbusClientError:
            # No cellular service or no SIM — not an error
            return

        status = DiagnosticStatus()
        status.name = f'Teltonika: {self.hardware_id}: cellular'
        status.hardware_id = self.hardware_id

        net_mode = signal.get('net_mode', 'No service')
        if net_mode == 'No service':
            status.level = DiagnosticStatus.WARN
            status.message = 'No service'
        else:
            status.level = DiagnosticStatus.OK
            status.message = net_mode

        for field in ['net_mode', 'rssi', 'rsrp', 'sinr', 'rsrq']:
            if field in signal:
                status.values.append(
                    KeyValue(key=field, value=str(signal[field]))
                )

        msg.status.append(status)

    def _add_mwan3_diagnostics(self, msg: DiagnosticArray):
        try:
            mwan = self.client.get_mwan3_status()
        except UbusClientError:
            return

        interfaces = mwan.get('interfaces', {})
        for name, data in interfaces.items():
            status = DiagnosticStatus()
            status.name = (
                f'Teltonika: {self.hardware_id}: mwan3/{name}'
            )
            status.hardware_id = self.hardware_id

            wan_status = data.get('status', 'unknown')
            if wan_status == 'online':
                status.level = DiagnosticStatus.OK
                status.message = 'Online'
            elif wan_status == 'offline':
                status.level = DiagnosticStatus.ERROR
                status.message = 'Offline'
            else:
                status.level = DiagnosticStatus.WARN
                status.message = wan_status

            for field in ['status', 'online', 'offline', 'uptime',
                          'score', 'lost', 'enabled', 'running', 'up']:
                if field in data:
                    status.values.append(
                        KeyValue(key=field, value=str(data[field]))
                    )

            # Track IP health
            for track in data.get('track_ip', []):
                ip = track.get('ip', '?')
                track_status = track.get('status', '?')
                latency = track.get('latency', 0)
                loss = track.get('packetloss', 0)
                status.values.append(
                    KeyValue(
                        key=f'track/{ip}',
                        value=f'{track_status} latency={latency} loss={loss}',
                    )
                )

            msg.status.append(status)

    def _add_interface_diagnostics(self, msg: DiagnosticArray):
        try:
            result = self.client.get_network_interfaces()
        except UbusClientError:
            return

        for iface in result.get('interface', []):
            name = iface.get('interface', 'unknown')
            status = DiagnosticStatus()
            status.name = (
                f'Teltonika: {self.hardware_id}: interface/{name}'
            )
            status.hardware_id = self.hardware_id

            is_up = iface.get('up', False)
            if is_up:
                status.level = DiagnosticStatus.OK
                status.message = 'Up'
            else:
                status.level = DiagnosticStatus.WARN
                status.message = 'Down'

            for field in ['up', 'proto', 'device', 'metric',
                          'uptime', 'l3_device']:
                if field in iface:
                    status.values.append(
                        KeyValue(key=field, value=str(iface[field]))
                    )

            # IP addresses
            for addr_info in iface.get('ipv4-address', []):
                addr = addr_info.get('address', '')
                mask = addr_info.get('mask', '')
                if addr:
                    status.values.append(
                        KeyValue(key='ipv4', value=f'{addr}/{mask}')
                    )

            msg.status.append(status)


def main(args=None):
    rclpy.init(args=args)
    node = TeltonikaMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
