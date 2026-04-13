# ros2_network_monitor

ROS 2 network monitoring tools — Teltonika routers, MikroTik devices, and generic network diagnostics.

All packages publish to `diagnostic_msgs/DiagnosticArray` for integration with standard ROS 2 diagnostics tooling.

## Packages

- **teltonika_monitor** — Teltonika router monitoring (cellular signal, mwan3, VPN, interface stats)
- **mikrotik_monitor** — MikroTik device monitoring (WiFi signal, traffic, association)
- **network_tools** — Generic network health (ping latency, packet loss, link up/down)

## Status

Under development. See individual package issues for progress.
