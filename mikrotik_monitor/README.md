# mikrotik_monitor

ROS 2 node that polls a MikroTik device via the RouterOS 7 REST API and
publishes network status to `diagnostic_msgs/DiagnosticArray`.

Zero external Python dependencies — uses only `urllib` (stdlib).

## Published Topics

- `/diagnostics` (`diagnostic_msgs/DiagnosticArray`) — interface status and
  wireless registration data

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `host` | string | `''` | MikroTik device hostname or IP (required) |
| `username` | string | `'admin'` | RouterOS username |
| `password` | string | `''` | RouterOS password |
| `port` | int | `80` | REST API port |
| `use_ssl` | bool | `false` | Use HTTPS instead of HTTP |
| `poll_interval` | double | `5.0` | Polling interval in seconds |
| `hardware_id` | string | `''` | Hardware ID for diagnostics (defaults to host) |
| `verify_ssl` | bool | `false` | Verify SSL certificate (when use_ssl is true) |

## Diagnostics Output

Each poll publishes a `DiagnosticArray` containing:

- **One `DiagnosticStatus` per network interface** — type, running/disabled
  state, tx/rx bytes, packets, errors, drops, link-downs
- **One `DiagnosticStatus` per wireless registration** — signal strength,
  signal-to-noise ratio, tx/rx rate, CCQ, uptime
- **One `DiagnosticStatus` per wireless radio interface** (`events/<iface>`)
  — surfaces association / disassociation / deauth events from the
  RouterOS log buffer. Fields: `last_assoc`, `last_drop` (with reason),
  `drops_last_5min`, `drops_last_60min`, `current_session_age_sec`.
  Level is always **OK** as long as the underlying poll is healthy:
  wireless drops are routine for marine ops (over-horizon / out-of-range
  is a valid mode of operation), so this task surfaces the data
  unjudged. Downstream annunciators with range awareness or
  bag-time forensics can apply context.

## Launch

```bash
ros2 launch mikrotik_monitor mikrotik_monitor.launch.py \
    config_file:=/path/to/your/config.yaml
```

Or run directly:

```bash
ros2 run mikrotik_monitor mikrotik_monitor_node --ros-args \
    -p host:=<device-ip> \
    -p username:=ros_monitor \
    -p password:=<password>
```

## Deployment

Run one node per MikroTik device, monitoring only the **local** device on
each machine. This avoids duplicate diagnostics when messages are bridged
(e.g., via `udp_bridge`) and ensures each node can always reach its device
regardless of WiFi link state.

For example, with a WiFi bridge between an operator station and a boat:

- **Operator station** runs one node monitoring the operator-side MikroTik
- **Boat** runs one node monitoring the boat-side MikroTik

If the WiFi link drops, each side still reports its local device status.
The wireless registration table going empty is itself an indicator that
the link is down.

## MikroTik Device Setup

Create a read-only monitoring account on each MikroTik device. This limits
API access to read-only operations.

### Using WebFig (web GUI)

1. **Create user group**: Go to **System → Users → Groups tab → Add New (+)**.
   Set name to `ros_monitor`. Check only `read`, `api`, and `rest-api`
   policies — uncheck everything else. Click **OK**.

2. **Create user**: Go to **System → Users → Users tab → Add New (+)**. Set
   name to `ros_monitor`, group to `ros_monitor`, and choose a password.
   Click **OK**.

3. **Verify REST API is enabled**: Go to **IP → Services**. Confirm `www`
   (port 80) is enabled.

### Using CLI

```
/user/group/add name=ros_monitor policy=read,api,rest-api,!write,!policy,!ftp,!ssh,!reboot,!local,!telnet,!winbox
/user/add name=ros_monitor group=ros_monitor password=<password>
```

### Verify

```bash
curl -u ros_monitor:<password> http://<device-ip>/rest/system/identity
```

Should return `{"name":"MikroTik"}`.
