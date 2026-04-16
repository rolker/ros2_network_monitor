"""ROS 2 node that polls a Teltonika router and publishes diagnostics."""

import threading
from datetime import datetime, timezone

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
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
        self.declare_parameter('publish_interval', 1.0)
        self.declare_parameter('hardware_id', '')
        self.declare_parameter('verify_ssl', False)
        self.declare_parameter(
            'ignored_interfaces', rclpy.Parameter.Type.STRING_ARRAY
        )

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
        # Denylist of interface names (applies to both network interface
        # and mwan3 diagnostics) — used to suppress noise from
        # intentionally unused interfaces.
        self._ignored_interfaces = set(
            self.get_parameter(
                'ignored_interfaces'
            ).get_parameter_value().string_array_value
        )

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

        # Cache of the most recent poll result (success or failure);
        # republished at publish_interval to avoid false STALE in
        # downstream consumers. On client error the cached message
        # carries the ERROR status so subscribers see the router is
        # down persistently between retries.
        self._cached_msg: DiagnosticArray | None = None
        self._cache_lock = threading.Lock()

        # Separate callback groups so the publish timer can run while a
        # slow poll is in progress — required by the MultiThreadedExecutor
        # in main(). Multiple sequential ubus JSON-RPC calls per poll
        # can stall for tens of seconds when the router is unreachable;
        # without the split the publish_interval guarantee would be
        # broken exactly when downstream consumers most need fresh data.
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
            f'Monitoring Teltonika router at {host}:{port} '
            f'poll every {poll_interval}s, '
            f'publish every {publish_interval}s'
        )

    def poll_callback(self):
        msg = DiagnosticArray()
        query_time_iso = datetime.now(timezone.utc).isoformat()

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

        # Stamp every status with the actual query time so observers can
        # compute true data age across republishes.
        for status in msg.status:
            status.values.append(
                KeyValue(key='last_query_time', value=query_time_iso)
            )
        with self._cache_lock:
            self._cached_msg = msg

    def _publish_callback(self):
        with self._cache_lock:
            msg = self._cached_msg
        if msg is None:
            return
        # Refresh send timestamp; per-status last_query_time is left intact.
        msg.header.stamp = self.get_clock().now().to_msg()
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
                status.values.append(KeyValue(
                    key=f'release.{field}',
                    value=str(release[field]),
                ))

        msg.status.append(status)

    @staticmethod
    def _parse_dbm(value):
        """Parse a dBm string like '-85 dBm' to a float, or return None."""
        if value is None:
            return None
        try:
            return float(str(value).split()[0])
        except (ValueError, IndexError):
            return None

    @staticmethod
    def _parse_db(value):
        """Parse a dB string like '12.5 dB' to a float, or return None."""
        if value is None:
            return None
        try:
            return float(str(value).split()[0])
        except (ValueError, IndexError):
            return None

    @staticmethod
    def _cellular_quality(rsrp, sinr):
        """Map RSRP/SINR to a diagnostic level and signal bars string.

        Thresholds based on 3GPP signal quality ranges for LTE:
          Excellent: RSRP > -80   SINR > 20
          Good:      RSRP > -90   SINR > 13
          Fair:      RSRP > -100  SINR > 0
          Poor:      RSRP > -110  SINR > -5
          Very poor: below
        """
        bar_chars = ['▁', '▂', '▃', '▅', '█']
        if rsrp is None:
            return DiagnosticStatus.WARN, '?'
        if rsrp > -80:
            n = 5
        elif rsrp >= -90:
            n = 4
        elif rsrp >= -100:
            n = 3
        elif rsrp >= -110:
            n = 2
        else:
            n = 1
        # SINR can downgrade by one bar
        if sinr is not None and sinr < 0 and n > 1:
            n -= 1
        bars = ''.join(bar_chars[:n]) + ''.join('·' for _ in range(5 - n))
        if n >= 4:
            level = DiagnosticStatus.OK
        elif n >= 2:
            level = DiagnosticStatus.WARN
        else:
            level = DiagnosticStatus.ERROR
        return level, bars

    def _add_cellular_diagnostics(self, msg: DiagnosticArray):
        try:
            signal = self.client.get_signal()
        except UbusClientError as e:
            if e.code == 3:
                # Method not found — no cellular modem
                return
            self.get_logger().warning(
                f'Failed to query cellular signal: {e}'
            )
            return

        status = DiagnosticStatus()
        status.name = f'Teltonika: {self.hardware_id}: cellular'
        status.hardware_id = self.hardware_id

        net_mode = signal.get('net_mode', 'No service')
        if net_mode == 'No service':
            status.level = DiagnosticStatus.WARN
            status.message = 'No service'
        else:
            rsrp = self._parse_dbm(signal.get('rsrp'))
            sinr = self._parse_db(signal.get('sinr'))
            level, bars = self._cellular_quality(rsrp, sinr)
            status.level = level
            rsrp_str = f' {rsrp:.0f}dBm' if rsrp is not None else ''
            status.message = f'{net_mode}{rsrp_str} {bars}'

        for field in ['net_mode', 'rssi', 'rsrp', 'sinr', 'rsrq']:
            if field in signal:
                status.values.append(
                    KeyValue(key=field, value=str(signal[field]))
                )

        msg.status.append(status)

    def _add_mwan3_diagnostics(self, msg: DiagnosticArray):
        try:
            mwan = self.client.get_mwan3_status()
        except UbusClientError as e:
            if e.code == 3:
                # Method not found — mwan3 not installed
                return
            self.get_logger().warning(
                f'Failed to query mwan3 status: {e}'
            )
            return

        interfaces = mwan.get('interfaces', {})
        for name, data in interfaces.items():
            if name in self._ignored_interfaces:
                continue
            status = DiagnosticStatus()
            status.name = (
                f'Teltonika: {self.hardware_id}: mwan3/{name}'
            )
            status.hardware_id = self.hardware_id

            wan_status = data.get('status', 'unknown')
            if wan_status == 'online':
                status.level = DiagnosticStatus.OK
                status.message = 'Online'
            elif wan_status == 'standby':
                # Standby is the expected steady state for a configured
                # backup interface while the primary is up — not a warning.
                status.level = DiagnosticStatus.OK
                status.message = 'Standby'
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
        except UbusClientError as e:
            if e.code == 3:
                return
            self.get_logger().warning(
                f'Failed to query network interfaces: {e}'
            )
            return

        for iface in result.get('interface', []):
            name = iface.get('interface', 'unknown')
            if name in self._ignored_interfaces:
                continue
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
