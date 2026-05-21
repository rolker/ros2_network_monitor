# Copyright 2024 Roland Arsenault
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""
ROS 2 node that polls a MikroTik device and publishes diagnostics.

Uses ``diagnostic_updater.Updater`` so tasks publish at a steady cadence
regardless of how long the RouterOS REST calls take.  Poll callback only
updates a ``CachedStatus`` snapshot; per-task callbacks render
``DiagnosticStatus`` from the cache via
:mod:`mikrotik_monitor.diagnostics_logic`.

Two fixed tasks always exist (``: connection`` and ``: system``).
Per-interface and per-wireless-registration tasks are added and removed
dynamically via ``Updater.add`` / ``Updater.removeByName`` as the
observed membership set changes.  A grace period prevents churn on
transient flaps.
"""

from datetime import datetime, timezone
import threading
import time

import diagnostic_updater
from mikrotik_monitor.diagnostics_logic import (
    CachedStatus,
    clamp_backoff_max_sec,
    compute_poll_backoff,
    diff_dynamic_membership,
    event_interface,
    interface_task_name,
    synthesize_connection_status,
    synthesize_interface_status,
    synthesize_system_status,
    synthesize_wireless_events_status,
    synthesize_wireless_status,
    wireless_events_task_name,
    wireless_task_name,
)
from mikrotik_monitor.routeros_client import RouterOSClient, RouterOSClientError
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class MikroTikMonitorNode(Node):

    def __init__(self):
        super().__init__('mikrotik_monitor')

        self.declare_parameter('host', '')
        self.declare_parameter('username', 'admin')
        self.declare_parameter('password', '')
        self.declare_parameter('port', 80)
        self.declare_parameter('use_ssl', False)
        self.declare_parameter('poll_interval', 5.0)
        self.declare_parameter('update_period_sec', 1.0)
        self.declare_parameter('stale_timeout_sec', 0.0)
        self.declare_parameter('dynamic_task_grace_sec', 0.0)
        self.declare_parameter('hardware_id', '')
        self.declare_parameter('verify_ssl', False)
        self.declare_parameter('ignored_interfaces', [])
        # Cap on the exponential-backoff interval applied to the poll
        # timer body when the device is unreachable.  Backoff starts at
        # poll_interval and doubles per consecutive failure, capped here.
        # On any success the failure counter resets.  Issue #23.
        self.declare_parameter('backoff_max_sec', 60.0)

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

        self.client = RouterOSClient(
            host=host,
            username=username,
            password=password,
            port=port,
            use_ssl=use_ssl,
            verify_ssl=verify_ssl,
        )

        if not self.hardware_id:
            self.hardware_id = host

        # stale_timeout_sec = 0 → auto (3 × poll_interval).
        stale_timeout_sec = self.get_parameter(
            'stale_timeout_sec'
        ).get_parameter_value().double_value
        if stale_timeout_sec <= 0.0:
            stale_timeout_sec = 3.0 * poll_interval
        self._stale_timeout_sec = stale_timeout_sec

        # dynamic_task_grace_sec = 0 → auto (2 × stale_timeout_sec).
        # Interfaces/wireless registrations missing from a single poll
        # are given this long to reappear before Updater.removeByName is
        # called on them.  Prevents churn on transient flaps.  Field-
        # tunable; the first long deployment with the new nodes should
        # observe flap frequency and inform a better default.
        grace_sec = self.get_parameter(
            'dynamic_task_grace_sec'
        ).get_parameter_value().double_value
        if grace_sec <= 0.0:
            grace_sec = 2.0 * stale_timeout_sec
        self._dynamic_task_grace_sec = grace_sec

        self._name_prefix = f'MikroTik: {self.hardware_id}'

        # Poll-cadence + backoff state (issue #23).  ``poll_interval`` is
        # the timer period.  When the device is unreachable we don't
        # recreate the timer; instead we gate the poll body on
        # ``_next_poll_monotonic`` and double the gap per consecutive
        # failure, capped at ``backoff_max_sec``.  The Updater's separate
        # publish timer keeps emitting DiagnosticStatus.ERROR from the
        # cached error_message throughout the retry phase so the operator
        # never sees silence — closes the silent-startup-crash window
        # reported in the 2026-05-19 deployment.
        self._poll_interval = poll_interval
        backoff_max_sec, backoff_clamped = clamp_backoff_max_sec(
            self.get_parameter('backoff_max_sec')
            .get_parameter_value().double_value,
            poll_interval,
        )
        if backoff_clamped:
            self.get_logger().warning(
                f'backoff_max_sec was shorter than poll_interval '
                f'({poll_interval}s); clamped to {backoff_max_sec}s so the '
                f'cached (retry in Xs) hint stays accurate.'
            )
        self._backoff_max_sec = backoff_max_sec
        self._consecutive_failures = 0
        self._next_poll_monotonic = 0.0  # 0 → poll on next timer fire

        # Cache + dynamic-membership bookkeeping.  All access guarded by
        # _cache_lock so task callbacks on the rclpy thread only see
        # consistent snapshots.
        self._cache_lock = threading.Lock()
        self._cache = CachedStatus()
        # Registered dynamic-task names → last monotonic time observed.
        # Fed to diff_dynamic_membership to drive grace-period removal.
        self._iface_last_seen: dict[str, float] = {}
        self._wireless_last_seen: dict[str, float] = {}
        self._event_iface_last_seen: dict[str, float] = {}

        # diagnostic_updater.Updater publishes all registered tasks at
        # update_period_sec regardless of poll timing.
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

        # Separate callback groups: the Updater publish timer must run
        # while a slow poll is in progress.  RouterOSClient HTTP
        # requests can stall for tens of seconds when the device is
        # unreachable.
        self.poll_timer = self.create_timer(
            poll_interval,
            self._poll_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.get_logger().info(
            f'Monitoring MikroTik device at {host}:{port} '
            f'poll every {poll_interval}s, '
            f'publish every {update_period_sec}s, '
            f'stale_timeout={self._stale_timeout_sec}s, '
            f'dynamic_task_grace={self._dynamic_task_grace_sec}s, '
            f'backoff_max={self._backoff_max_sec}s'
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

    def _make_wireless_task(self, iface: str, mac: str):
        def _task(stat):
            level, message, kvs = synthesize_wireless_status(
                self._cache_snapshot(),
                iface,
                mac,
                self._stale_timeout_sec,
                time.monotonic(),
            )
            return _emit(stat, level, message, kvs)
        return _task

    def _make_events_task(self, iface: str):
        def _task(stat):
            level, message, kvs = synthesize_wireless_events_status(
                self._cache_snapshot(),
                iface,
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
        """Fetch the latest device state; update cache and dynamic tasks.

        Wrapped end-to-end so no callback exception can escape into the
        executor and silently kill the node.  Connection-class failures
        come through as ``RouterOSClientError`` and feed the normal
        cache-preservation path (DiagnosticStatus ERROR via the Updater);
        any other exception type is caught at the outer ``Exception``
        guard, logged, and surfaced the same way.  Either form bumps the
        consecutive-failure counter, which exponentially backs off the
        next poll attempt up to ``backoff_max_sec``.  See issue #23.
        """
        now = time.monotonic()
        if now < self._next_poll_monotonic:
            # In backoff after a previous failure; skip this fire.
            # The Updater's publish timer continues emitting ERROR via
            # the cached error_message so the operator stays informed.
            return

        poll_wall_iso = datetime.now(timezone.utc).isoformat()
        error_message = None
        system_resource = None
        system_health = None
        interfaces: list[dict] | None = None   # None → inherit prior
        wireless: list[dict] | None = None     # None → inherit prior
        wireless_events: list[dict] | None = None   # None → inherit prior

        try:
            system_resource = self.client.get_system_resource()
            # Health is optional — not all devices support it.
            try:
                system_health = self.client.get_system_health()
            except RouterOSClientError:
                system_health = None
            # Wrap get_interfaces individually for symmetry with the
            # other sub-queries — a partial failure here should not
            # discard the already-successful system_resource.
            try:
                raw_interfaces = self.client.get_interfaces()
                interfaces = [
                    i for i in raw_interfaces
                    if i.get('name', 'unknown') not in self._ignored_interfaces
                ]
            except RouterOSClientError as e:
                self.get_logger().warning(
                    f'Failed to query interfaces: {e}'
                )
                interfaces = None  # sentinel: "don't know"; preserve prior
            try:
                raw_wireless = self.client.get_wireless_registrations()
                wireless = [
                    w for w in raw_wireless
                    if w.get('interface') not in self._ignored_interfaces
                ]
            except RouterOSClientError as e:
                if e.http_code in (400, 404):
                    # No wireless interfaces on this device — not an error.
                    wireless = []
                else:
                    self.get_logger().warning(
                        f'Failed to query wireless registrations: {e}'
                    )
                    wireless = None  # sentinel: preserve prior
            # Wireless event log — used by the per-radio events/<iface>
            # diagnostic task.  RouterOSClient.get_log() fetches the full
            # /log buffer and filters by topic substring in Python
            # (RouterOS REST topic-filter semantics are inconsistent
            # across versions, see routeros_client.get_log docstring).
            # Worth revisiting if log volume becomes a perf concern on
            # busy devices.
            try:
                wireless_events = self.client.get_log(topics_filter='wireless')
            except RouterOSClientError as e:
                self.get_logger().warning(
                    f'Failed to query log: {e}'
                )
                wireless_events = None  # sentinel: preserve prior
        except RouterOSClientError as e:
            error_message = f'Connection error: {e}'
            self.get_logger().warning(f'Failed to poll device: {e}')
        except Exception as e:
            # Catch-all: an unexpected exception type (e.g., a stdlib
            # change, an unforeseen payload shape, an SSL surprise) must
            # not escape the timer callback — that's the silent-startup-
            # crash path from issue #23.  Surface the error the same way
            # a connection failure would, so the operator sees ERROR on
            # /diagnostics instead of nothing.
            error_message = f'Unexpected error: {type(e).__name__}: {e}'
            self.get_logger().error(
                f'Unexpected error polling device: {e}', exc_info=True
            )

        # Update backoff bookkeeping.  Done before cache mutation so that
        # the error_message we cache can include the retry-in-Ns hint.
        if error_message is None:
            if self._consecutive_failures > 0:
                self.get_logger().info(
                    f'Recovered after {self._consecutive_failures} '
                    f'failed poll attempt(s).'
                )
            self._consecutive_failures = 0
            self._next_poll_monotonic = 0.0
        else:
            self._consecutive_failures += 1
            backoff = compute_poll_backoff(
                self._poll_interval,
                self._consecutive_failures,
                self._backoff_max_sec,
            )
            self._next_poll_monotonic = time.monotonic() + backoff
            # Augment the cached error message with the backoff window
            # so the operator can see we're in a backoff phase, not
            # stuck.  The poll body early-returns until
            # ``_next_poll_monotonic`` so this cached string sticks
            # around for the full window — phrased as the window itself
            # (``backoff Xs``) rather than a countdown (``retry in Xs``)
            # so the value stays accurate without per-timer-fire cache
            # refreshes.
            error_message = (
                f'{error_message} '
                f'(attempt {self._consecutive_failures}, '
                f'backoff {backoff:.0f}s)'
            )

        reconcile_args: tuple | None = None
        with self._cache_lock:
            prev = self._cache
            poll_now = time.monotonic()
            if error_message is None:
                # Successful outer poll.  Use freshly-fetched fields,
                # substituting the previous cache value for any sub-query
                # that returned None (sentinel from inner except).
                # poll_monotonic advances to "now" because we have fresh
                # confirmation of device reachability.
                #
                # wireless_events_last_success_monotonic only advances
                # when the /log fetch itself succeeded — that way the
                # events/<iface> task surfaces STALE even if the outer
                # poll keeps succeeding while /log keeps failing.
                new_cache = CachedStatus(
                    system_resource=system_resource,
                    system_health=(
                        system_health if system_health is not None
                        else prev.system_health
                    ),
                    interfaces=(
                        interfaces if interfaces is not None
                        else prev.interfaces
                    ),
                    wireless=(
                        wireless if wireless is not None
                        else prev.wireless
                    ),
                    wireless_events=(
                        wireless_events if wireless_events is not None
                        else prev.wireless_events
                    ),
                    poll_monotonic=poll_now,
                    poll_wall_iso=poll_wall_iso,
                    system_health_last_success_monotonic=(
                        poll_now if system_health is not None
                        else prev.system_health_last_success_monotonic
                    ),
                    interfaces_last_success_monotonic=(
                        poll_now if interfaces is not None
                        else prev.interfaces_last_success_monotonic
                    ),
                    wireless_last_success_monotonic=(
                        poll_now if wireless is not None
                        else prev.wireless_last_success_monotonic
                    ),
                    wireless_events_last_success_monotonic=(
                        poll_now if wireless_events is not None
                        else prev.wireless_events_last_success_monotonic
                    ),
                    error_message=None,
                )
                # Capture what _reconcile_dynamic_tasks needs BEFORE
                # releasing the lock so the reconcile call sees a
                # consistent snapshot even if another poll is already
                # in flight on another thread.  Only captured on the
                # success path — failure skips reconciliation entirely.
                reconcile_args = (
                    new_cache.interfaces,
                    new_cache.wireless,
                    new_cache.wireless_events,
                    new_cache.poll_monotonic,
                )
            else:
                # Outer connection failure.  Preserve previous cache
                # fields so per-task callbacks keep rendering last-known
                # data until the cache ages past stale_timeout_sec.
                # poll_monotonic stays at the previous successful value
                # (drives STALE emission).  poll_wall_iso updates so the
                # last_query_time KeyValue reflects the most recent poll
                # *attempt*.
                new_cache = CachedStatus(
                    system_resource=prev.system_resource,
                    system_health=prev.system_health,
                    interfaces=prev.interfaces,
                    wireless=prev.wireless,
                    wireless_events=prev.wireless_events,
                    poll_monotonic=prev.poll_monotonic,
                    poll_wall_iso=poll_wall_iso,
                    system_health_last_success_monotonic=(
                        prev.system_health_last_success_monotonic
                    ),
                    interfaces_last_success_monotonic=(
                        prev.interfaces_last_success_monotonic
                    ),
                    wireless_last_success_monotonic=(
                        prev.wireless_last_success_monotonic
                    ),
                    wireless_events_last_success_monotonic=(
                        prev.wireless_events_last_success_monotonic
                    ),
                    error_message=error_message,
                )
            self._cache = new_cache

        # Dynamic-membership reconciliation only runs on a successful
        # outer poll — on failure we don't know the current membership,
        # so leave the registered tasks alone and let them emit STALE
        # via their own callbacks once the preserved cache ages out.
        if reconcile_args is not None:
            self._reconcile_dynamic_tasks(*reconcile_args)

    def _reconcile_dynamic_tasks(
        self,
        interfaces: list[dict],
        wireless: list[dict],
        wireless_events: list[dict],
        now_monotonic: float,
    ):
        """Add/remove Updater tasks to match the observed membership.

        Called from ``_poll_callback`` on the poll timer's callback
        group, while the Updater's internal periodic publish timer
        runs in the node's default group.  Concurrent execution is
        safe: ``diagnostic_updater.Updater`` serializes ``add``,
        ``removeByName``, and ``update`` (task iteration) under a
        shared ``threading.Lock`` — see
        ``diagnostic_updater/_diagnostic_updater.py:172,199,213,274``
        in Jazzy.  Lock ordering in this node is clean: the cache
        lock is released before the Updater's internal lock is
        acquired, so no deadlock is possible.
        """
        observed_iface_names = {
            interface_task_name(self._name_prefix, i.get('name', 'unknown'))
            for i in interfaces
        }
        observed_wireless_names = {
            wireless_task_name(
                self._name_prefix,
                w.get('interface', 'unknown'),
                w.get('mac-address', 'unknown'),
            )
            for w in wireless
        }

        # Interfaces
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

        # Wireless
        to_add, to_remove, new_last_seen = diff_dynamic_membership(
            observed_wireless_names,
            set(self._wireless_last_seen.keys()),
            self._wireless_last_seen,
            self._dynamic_task_grace_sec,
            now_monotonic,
        )
        for task_name in to_add:
            # Task name is "<prefix>: wireless/<iface>/<mac>"
            suffix = task_name[len(self._name_prefix) + len(': wireless/'):]
            iface, _, mac = suffix.partition('/')
            self._updater.add(task_name, self._make_wireless_task(iface, mac))
        for task_name in to_remove:
            self._updater.removeByName(task_name)
        self._wireless_last_seen = new_last_seen

        # Wireless events — per-radio task scoped by interface name.
        # Membership union: any interface seen in the current wireless
        # registration table OR in the log buffer.  This means a radio
        # gets an event task once it has either an active peer or any
        # logged event — covers steady-state (peer associated) and
        # post-drop (peer gone but events remain in log).
        observed_event_ifaces = {
            w.get('interface', 'unknown') for w in wireless
        } | {
            iface for e in wireless_events
            if (iface := event_interface(e)) is not None
        }
        observed_event_ifaces.discard('unknown')
        observed_event_iface_names = {
            wireless_events_task_name(self._name_prefix, iface)
            for iface in observed_event_ifaces
            if iface not in self._ignored_interfaces
        }
        to_add, to_remove, new_last_seen = diff_dynamic_membership(
            observed_event_iface_names,
            set(self._event_iface_last_seen.keys()),
            self._event_iface_last_seen,
            self._dynamic_task_grace_sec,
            now_monotonic,
        )
        for task_name in to_add:
            iface_name = task_name[len(self._name_prefix) + len(': events/'):]
            self._updater.add(
                task_name, self._make_events_task(iface_name),
            )
        for task_name in to_remove:
            self._updater.removeByName(task_name)
        self._event_iface_last_seen = new_last_seen


def _emit(stat, level: int, message: str, kvs):
    """Apply (level, message, kvs) to a DiagnosticStatusWrapper."""
    stat.summary(level, message)
    for kv in kvs:
        stat.add(kv.key, kv.value)
    return stat


def main(args=None):
    rclpy.init(args=args)
    # Guard node construction so a parameter-validation or other init
    # failure is logged on the way out instead of vanishing into a bare
    # traceback on stdout (issue #23 — silent-startup-crash class).
    try:
        node = MikroTikMonitorNode()
    except SystemExit:
        rclpy.try_shutdown()
        raise
    except Exception as exc:
        # Use the rclpy logger so the failure lands on /rosout as well
        # as stderr.  rclpy.logging.get_logger works even before any
        # Node exists.  Capture the exception type+message so /rosout
        # shows *why* the node died, not just that it did — the bare
        # traceback only reaches stderr.
        rclpy.logging.get_logger('mikrotik_monitor').fatal(
            f'Failed to construct MikroTikMonitorNode: '
            f'{type(exc).__name__}: {exc}'
        )
        rclpy.try_shutdown()
        raise
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
