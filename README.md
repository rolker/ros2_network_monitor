# ros2_network_monitor

ROS 2 network monitoring tools — Teltonika routers, MikroTik devices, and generic network diagnostics.

All packages publish to `/diagnostics` via [`diagnostic_updater.Updater`](https://docs.ros.org/en/jazzy/p/diagnostic_updater/)
at a steady cadence (`update_period_sec`, default 1 Hz), independent of how long the
underlying poll calls take.  Polling is decoupled: the poll timer only updates an
internal cache; Updater task callbacks render `DiagnosticStatus` from the cache and
emit `DiagnosticStatus.STALE` when the cache ages past `stale_timeout_sec`.

## Packages

- **teltonika_monitor** — Teltonika router monitoring (cellular signal, mwan3, VPN, interface stats)
- **mikrotik_monitor** — MikroTik device monitoring (WiFi signal, traffic, association)
- **network_tools** — Generic network health (ping latency, packet loss, link up/down)

## Diagnostic naming

Each monitor emits a `<Prefix>: <hardware_id>: <task>` status per Updater task.
Fixed tasks are always registered; per-entity tasks (interfaces, wireless
registrations, mwan3 members) are added/removed dynamically via
`Updater.add` / `Updater.removeByName` with a configurable grace period
(`dynamic_task_grace_sec`, default `2 × stale_timeout_sec`) to absorb
transient flaps.

| Package | Fixed tasks | Dynamic tasks |
|---|---|---|
| `network_tools` (ping) | one per configured target | — |
| `mikrotik_monitor` | `: connection`, `: system` | `: interface/<name>`, `: wireless/<iface>/<mac>` |
| `teltonika_monitor` | `: connection`, `: system`, `: cellular` (optional) | `: mwan3/<name>`, `: interface/<name>` |

The `: connection` task reflects poll-reachability: `OK` when the last poll
succeeded, `ERROR` with the connection error message otherwise.  It replaces
an earlier error-path pattern where the monitor renamed its output to a
bare `<Prefix>: <hardware_id>` summary on connection failure — that pattern
orphaned per-task entries in downstream aggregators when connection state
flipped.  See [issue #15](https://github.com/rolker/ros2_network_monitor/issues/15).

## Status

Under development. See individual package issues for progress.
