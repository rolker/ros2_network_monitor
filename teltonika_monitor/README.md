# teltonika_monitor

ROS 2 node that polls a Teltonika router (e.g., RUTX11) via the ubus
JSON-RPC API over HTTPS and publishes network status to
`diagnostic_msgs/DiagnosticArray`.

Zero external Python dependencies — uses only `urllib` (stdlib).

## Published Topics

- `/diagnostics` (`diagnostic_msgs/DiagnosticArray`) — system, cellular,
  mwan3, and interface status

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `host` | string | `''` | Router hostname or IP (required) |
| `username` | string | `'ros_monitor'` | ubus JSON-RPC username |
| `password` | string | `''` | ubus JSON-RPC password |
| `port` | int | `443` | HTTPS port |
| `use_ssl` | bool | `true` | Use HTTPS (Teltonika redirects HTTP to HTTPS) |
| `poll_interval` | double | `5.0` | Polling interval in seconds |
| `hardware_id` | string | `''` | Hardware ID for diagnostics (defaults to host) |
| `verify_ssl` | bool | `false` | Verify SSL certificate |

## Diagnostics Output

Each poll publishes a `DiagnosticArray` containing:

- **System** — model, hostname, kernel version, firmware
- **Cellular** — network mode (LTE/3G), RSSI, RSRP, SINR, RSRQ
- **mwan3** — per-WAN-interface failover status, uptime, tracking IP
  health (latency, packet loss)
- **Interfaces** — up/down state, protocol, IP addresses

## Launch

```bash
ros2 launch teltonika_monitor teltonika_monitor.launch.py \
    config_file:=/path/to/your/config.yaml
```

Or run directly:

```bash
ros2 run teltonika_monitor teltonika_monitor_node --ros-args \
    -p host:=router.op \
    -p username:=ros_monitor \
    -p password:=<password>
```

## Deployment

Run one node per router, monitoring only the **local** router on each
machine. This avoids duplicate diagnostics when messages are bridged
(e.g., via `udp_bridge`).

## Router Setup

The node authenticates via ubus JSON-RPC, which requires a user account
with appropriate ACL permissions.

### 1. Create monitoring user (WebFig)

Go to **System → Administration → Access Control → System Users**:

1. **Add Group**: name `ros_monitor`
2. **Add User**: name `ros_monitor`, group `ros_monitor`, set password

### 2. Create ubus ACL file (SSH)

The web UI grants Teltonika REST API access but not ubus access. Create
an ACL file via SSH to grant read-only ubus access:

```bash
cat > /usr/share/rpcd/acl.d/ros_monitor.json << 'EOF'
{
        "ros_monitor": {
                "description": "Read-only access for ROS 2 monitoring",
                "read": {
                        "ubus": {
                                "system": ["board", "info"],
                                "gsm": ["*"],
                                "gsm.modem0": ["*"],
                                "network.interface": ["dump", "status"],
                                "mwan3": ["status"],
                                "wireguard": ["*"],
                                "gpsd": ["*"],
                                "ntp": ["*"]
                        }
                }
        }
}
EOF
/etc/init.d/rpcd restart
```

### 3. Verify

```bash
# Login (get session token)
curl -sk -d '{"jsonrpc":"2.0","id":1,"method":"call","params":["00000000000000000000000000000000","session","login",{"username":"ros_monitor","password":"<password>"}]}' https://<router-ip>/ubus

# Test ubus call (use session token from login response)
curl -sk -d '{"jsonrpc":"2.0","id":1,"method":"call","params":["<session>","system","board",{}]}' https://<router-ip>/ubus
```

## Compatibility

- Tested on RUTX11 with firmware RUTX_R_00.07.21.2
- Should work with other Teltonika RutOS devices that expose ubus over
  uhttpd (requires `uhttpd-mod-ubus` package)
