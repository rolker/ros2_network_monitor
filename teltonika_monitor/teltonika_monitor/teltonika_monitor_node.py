# Copyright 2024 Roland Arsenault
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""
ROS 2 node that polls a Teltonika router and publishes diagnostics.

Uses ``diagnostic_updater.Updater`` so tasks publish at a steady cadence
regardless of how long the sequence of ubus JSON-RPC calls takes.  Poll
callback only updates a ``CachedStatus`` snapshot; per-task callbacks
render ``DiagnosticStatus`` from the cache via
:mod:`teltonika_monitor.diagnostics_logic`.

Fixed tasks: ``: connection``, ``: system``, and ``: cellular`` (when
``publish_cellular=True``).  Per-mwan3-member and per-interface tasks
are added/removed dynamically via ``Updater.add`` / ``Updater.removeByName``
with a grace period to absorb transient flaps.
"""

from datetime import datetime, timezone
import threading
import time

import diagnostic_updater
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from teltonika_monitor.diagnostics_logic import (
    CachedStatus,
    diff_dynamic_membership,
    interface_task_name,
    mwan3_task_name,
    synthesize_cellular_status,
    synthesize_connection_status,
    synthesize_interface_status,
    synthesize_mwan3_status,
    synthesize_system_status,
)
from teltonika_monitor.ubus_client import UbusClient, UbusClientError


class TeltonikaMonitorNode(Node):

    def __init__(self):
        super().__init__('teltonika_monitor')

        self.declare_parameter('host', '')
        self.declare_parameter('username', 'ros_monitor')
        self.declare_parameter('password', '')
        self.declare_parameter('port', 443)
        self.declare_parameter('use_ssl', True)
        self.declare_parameter('poll_interval', 5.0)
        self.declare_parameter('update_period_sec', 1.0)
        self.declare_parameter('stale_timeout_sec', 0.0)
        self.declare_parameter('dynamic_task_grace_sec', 0.0)
        self.declare_parameter('hardware_id', '')
        self.declare_parameter('verify_ssl', False)
        self.declare_parameter('ignored_interfaces', [])
        self.declare_parameter('publish_cellular', True)

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
        update_period_sec = self.get_parameter(
            'update_period_sec'
        ).get_parameter_value().double_value
        self.hardware_id = self.get_parameter(
            'hardware_id'
        ).get_parameter_value().string_value
        verify_ssl = self.get_parameter(
            'verify_ssl'
        ).get_parameter_value().bool_value
        self._ignored_interfaces = set(
            self.get_parameter(
                'ignored_interfaces'
            ).get_parameter_value().string_array_value
        )
        self._publish_cellular = self.get_parameter(
            'publish_cellular'
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

        stale_timeout_sec = self.get_parameter(
            'stale_timeout_sec'
        ).get_parameter_value().double_value
        if stale_timeout_sec <= 0.0:
            stale_timeout_sec = 3.0 * poll_interval
        self._stale_timeout_sec = stale_timeout_sec

        # dynamic_task_grace_sec = 0 → auto (2 × stale_timeout_sec).
        # Field-tunable; first long deployment should observe
        # transient-flap frequency and inform a better default.
        grace_sec = self.get_parameter(
            'dynamic_task_grace_sec'
        ).get_parameter_value().double_value
        if grace_sec <= 0.0:
            grace_sec = 2.0 * stale_timeout_sec
        self._dynamic_task_grace_sec = grace_sec

        self._name_prefix = f'Teltonika: {self.hardware_id}'

        self._cache_lock = threading.Lock()
        self._cache = CachedStatus()
        self._mwan3_last_seen: dict[str, float] = {}
        self._iface_last_seen: dict[str, float] = {}

        self._updater = diagnostic_updater.Updater(
            self, period=update_period_sec,
        )
        self._updater.setHardwareID(self.hardware_id)

        # Fixed tasks — always registered.
        self._updater.add(
            f'{self._name_prefix}: connection',
            self._task_connection,
        )
        self._updater.add(
            f'{self._name_prefix}: system',
            self._task_system,
        )
        if self._publish_cellular:
            self._updater.add(
                f'{self._name_prefix}: cellular',
                self._task_cellular,
            )

        self.poll_timer = self.create_timer(
            poll_interval,
            self._poll_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.get_logger().info(
            f'Monitoring Teltonika router at {host}:{port} '
            f'poll every {poll_interval}s, '
            f'publish every {update_period_sec}s, '
            f'stale_timeout={self._stale_timeout_sec}s, '
            f'dynamic_task_grace={self._dynamic_task_grace_sec}s'
        )

    # --- Task callbacks ---

    def _task_connection(self, stat):
        level, message, kvs = synthesize_connection_status(
            self._cache_snapshot(),
            self._stale_timeout_sec,
            time.monotonic(),
        )
        return _emit(stat, level, message, kvs)

    def _task_system(self, stat):
        level, message, kvs = synthesize_system_status(
            self._cache_snapshot(),
            self._stale_timeout_sec,
            time.monotonic(),
        )
        return _emit(stat, level, message, kvs)

    def _task_cellular(self, stat):
        level, message, kvs = synthesize_cellular_status(
            self._cache_snapshot(),
            self._stale_timeout_sec,
            time.monotonic(),
        )
        return _emit(stat, level, message, kvs)

    def _make_mwan3_task(self, mwan_name: str):
        def _task(stat):
            level, message, kvs = synthesize_mwan3_status(
                self._cache_snapshot(),
                mwan_name,
                self._stale_timeout_sec,
                time.monotonic(),
            )
            return _emit(stat, level, message, kvs)
        return _task

    def _make_interface_task(self, iface_name: str):
        def _task(stat):
            level, message, kvs = synthesize_interface_status(
                self._cache_snapshot(),
                iface_name,
                self._stale_timeout_sec,
                time.monotonic(),
            )
            return _emit(stat, level, message, kvs)
        return _task

    def _cache_snapshot(self) -> CachedStatus:
        with self._cache_lock:
            return self._cache

    # --- Polling ---

    def _poll_callback(self):
        """Fetch the latest router state; update cache and dynamic tasks."""
        poll_wall_iso = datetime.now(timezone.utc).isoformat()
        error_message = None
        system_board = None
        cellular_signal = None
        cellular_supported = True
        mwan3 = None
        mwan3_supported = True
        network_interfaces: list[dict] = []

        try:
            system_board = self.client.get_system_board()

            if self._publish_cellular:
                try:
                    cellular_signal = self.client.get_signal()
                except UbusClientError as e:
                    if e.code == 3:
                        cellular_supported = False
                    else:
                        self.get_logger().warning(
                            f'Failed to query cellular signal: {e}'
                        )
                        cellular_signal = None

            try:
                mwan3 = self.client.get_mwan3_status()
            except UbusClientError as e:
                if e.code == 3:
                    mwan3_supported = False
                else:
                    self.get_logger().warning(
                        f'Failed to query mwan3 status: {e}'
                    )
                    mwan3 = None

            try:
                result = self.client.get_network_interfaces()
                network_interfaces = [
                    i for i in result.get('interface', [])
                    if i.get('interface', 'unknown') not in self._ignored_interfaces
                ]
            except UbusClientError as e:
                if e.code != 3:
                    self.get_logger().warning(
                        f'Failed to query network interfaces: {e}'
                    )
                network_interfaces = []
        except UbusClientError as e:
            error_message = f'Connection error: {e}'
            self.get_logger().warning(f'Failed to poll router: {e}')

        poll_monotonic = time.monotonic()
        new_cache = CachedStatus(
            system_board=system_board,
            cellular_signal=cellular_signal,
            cellular_supported=cellular_supported,
            mwan3=mwan3,
            mwan3_supported=mwan3_supported,
            network_interfaces=network_interfaces,
            poll_monotonic=poll_monotonic,
            poll_wall_iso=poll_wall_iso,
            error_message=error_message,
        )
        with self._cache_lock:
            self._cache = new_cache

        if error_message is None:
            self._reconcile_dynamic_tasks(
                mwan3, network_interfaces, poll_monotonic,
            )

    def _reconcile_dynamic_tasks(
        self,
        mwan3: dict | None,
        network_interfaces: list[dict],
        now_monotonic: float,
    ):
        """Add/remove Updater tasks to match observed mwan3 / interface membership."""
        # mwan3 members — filter by ignored_interfaces (mwan3 names
        # overlap with interface names).
        observed_mwan_names: set[str] = set()
        if mwan3 is not None:
            for name in mwan3.get('interfaces', {}).keys():
                if name in self._ignored_interfaces:
                    continue
                observed_mwan_names.add(mwan3_task_name(self._name_prefix, name))

        to_add, to_remove, new_last_seen = diff_dynamic_membership(
            observed_mwan_names,
            set(self._mwan3_last_seen.keys()),
            self._mwan3_last_seen,
            self._dynamic_task_grace_sec,
            now_monotonic,
        )
        for task_name in to_add:
            mwan_name = task_name[len(self._name_prefix) + len(': mwan3/'):]
            self._updater.add(task_name, self._make_mwan3_task(mwan_name))
        for task_name in to_remove:
            self._updater.removeByName(task_name)
        self._mwan3_last_seen = new_last_seen

        # Network interfaces (already filtered by ignored_interfaces
        # during _poll_callback).
        observed_iface_names = {
            interface_task_name(self._name_prefix, i.get('interface', 'unknown'))
            for i in network_interfaces
        }
        to_add, to_remove, new_last_seen = diff_dynamic_membership(
            observed_iface_names,
            set(self._iface_last_seen.keys()),
            self._iface_last_seen,
            self._dynamic_task_grace_sec,
            now_monotonic,
        )
        for task_name in to_add:
            iface_name = task_name[len(self._name_prefix) + len(': interface/'):]
            self._updater.add(task_name, self._make_interface_task(iface_name))
        for task_name in to_remove:
            self._updater.removeByName(task_name)
        self._iface_last_seen = new_last_seen


def _emit(stat, level: int, message: str, kvs):
    """Apply (level, message, kvs) to a DiagnosticStatusWrapper."""
    stat.summary(level, message)
    for kv in kvs:
        stat.add(kv.key, kv.value)
    return stat


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
