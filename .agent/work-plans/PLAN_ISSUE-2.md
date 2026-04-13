# Plan: mikrotik_monitor — MikroTik device monitoring package

## Issue

https://github.com/rolker/ros2_network_monitor/issues/2

## Context

The `ros2_network_monitor` repo currently contains only a README. This is the
first package to be created. BizzyBoat uses MikroTik devices for WiFi bridging
between boat and operator station. Both sides have MikroTik devices reachable
from the development machine for testing.

All devices run RouterOS 7 — no RouterOS 6 support needed (per issue comment).

## Approach

1. **Create ament_python package skeleton** — `mikrotik_monitor/` with standard
   structure: `package.xml`, `setup.py`, `setup.cfg`, `resource/` marker,
   module directory, launch file.

2. **Implement RouterOS REST client** (`routeros_client.py`) — minimal client
   using only `urllib.request` (stdlib) to query the RouterOS 7 REST API.
   Supports basic auth over HTTPS (self-signed certs). No pip dependencies.
   Key endpoints:
   - `GET /rest/interface` — all interfaces with traffic counters
   - `GET /rest/interface/wireless/registration-table` — WiFi link stats
     (signal, noise, tx/rx rate)

3. **Implement ROS 2 node** (`mikrotik_monitor_node.py`) — simple `Node` with
   timer-based polling. Parameters: `host`, `username`, `password`,
   `poll_interval`, `hardware_id`. Publishes `DiagnosticArray`:
   - One `DiagnosticStatus` per wireless registration (signal strength, noise
     floor, bitrate, association state)
   - One `DiagnosticStatus` per interface (bytes, packets, errors)
   - Level = ERROR on connection failure, WARN on degraded signal (threshold TBD
     during testing)

4. **Add launch file and default config** — `launch/mikrotik_monitor.launch.py`
   with `config/mikrotik_monitor.yaml` for default parameters.

5. **Test with real devices** — iterate on which interfaces/data are useful
   based on actual device responses. Adjust fields and thresholds accordingly.

6. **Document** — README with node description, parameters, topics, and
   recommended MikroTik user setup for monitoring credentials.

## Files to Change

| File | Change |
|------|--------|
| `mikrotik_monitor/package.xml` | New: ament_python package manifest |
| `mikrotik_monitor/setup.py` | New: Python package setup |
| `mikrotik_monitor/setup.cfg` | New: setuptools config |
| `mikrotik_monitor/resource/mikrotik_monitor` | New: empty marker |
| `mikrotik_monitor/mikrotik_monitor/__init__.py` | New: empty init |
| `mikrotik_monitor/mikrotik_monitor/routeros_client.py` | New: REST API client (stdlib only) |
| `mikrotik_monitor/mikrotik_monitor/mikrotik_monitor_node.py` | New: ROS 2 node |
| `mikrotik_monitor/launch/mikrotik_monitor.launch.py` | New: launch file |
| `mikrotik_monitor/config/mikrotik_monitor.yaml` | New: default params |
| `mikrotik_monitor/mikrotik_monitor/README.md` | New: package docs |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Only what's needed | Stdlib-only REST client, no unnecessary abstractions. Start with basic polling, iterate based on real device testing. |
| Improve incrementally | First package in repo. Start simple, add sophistication (thresholds, filtering) based on testing. |
| Test what breaks | Connection failures and degraded signal are the field-relevant failure modes. Unit tests for response parsing; integration testing against real devices. |
| A change includes its consequences | Update repo README to reflect package existence. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0008 (ROS 2 conventions) | Yes | Standard ament_python structure, diagnostic_msgs |
| ADR-0009 (Python packages) | Yes | Zero pip dependencies — stdlib only (urllib, hashlib, ssl) |
| ADR-0003 (project agnostic) | No | This is a project repo, not workspace infrastructure |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Add mikrotik_monitor package | Repo README (package list) | Yes |

## Open Questions

- Which interfaces are worth monitoring individually vs. aggregating? (Resolve during testing)
- What signal strength thresholds for WARN level? (Resolve during testing)
- Should we support monitoring multiple devices from one node, or one node per device? (Starting with one-per-device, simpler)

## Estimated Scope

Single PR. Package skeleton + REST client + node + launch — all tightly coupled,
no benefit to splitting.
