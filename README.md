# rv2_csm_topic_bridge — R1

The `r1` branch uses `rv2_control_signal_transport` R1 and `r1_interfaces`.
The legacy `master` branch is unchanged. Build with the transport `r1` branch;
legacy transport headers and `rv2_interfaces` control messages are incompatible.

The bridge subscribes immediately and registers a source while fresh input is
available. Joystick/Twist/String topic transport and Joystick/Twist service
transport are supported. Payload fields are forwarded unchanged through typed
`SourceHandle` APIs. Registration and service waits run on a dedicated worker,
so executor callbacks can deliver their responses.

## Run a physical joystick

Start these in terminals with the same ROS environment/domain:

```bash
ros2 run joy joy_node --ros-args -p autorepeat_rate:=20.0
ros2 launch rv2_csm_topic_bridge topic_bridge.launch.py
ros2 launch rv2_server_control control_server.launch.py
```

The bridge defaults to `/joy`, target manager `control_server`, controller
`local_xbox`, channel `topic_bridge_control`, and priority 80. A 20 Hz autorepeat
keeps a stationary connected joystick active. Moving the joystick should change
the server's output; stopping `joy_node` or unplugging the device should stop
new forwarded commands. The bridge emits `ACTIVE`, `TIMEOUT` and `DISCONNECTED`
source transitions. The server launch starts one `csm_master` by default; this
master is needed to reconcile a restarted server while topic input continues.
When another launch already owns the master, pass `start_master:=false` to the
server and use the same `master_name` everywhere. To run the server executable
directly, start the master separately:

```bash
ros2 run rv2_control_signal_transport csm_master_node
```

The bridge can start before the server and continues registration attempts while
fresh joystick input arrives. Stop the input for more than `timeout_ms` to see
TIMEOUT, then resume to recover. Stop it beyond `disconnect_timeout_ms` to remove
the source/sink, then resume: the bridge creates a fresh registration. Also stop
and restart the server while continuing joystick input to check master-driven
reconciliation. Hardware hotplug behavior of the joystick driver is external to
this package; if the driver stops reopening the device, restart `joy_node`.

Inspect the R1 status streams as needed:

```bash
ros2 topic echo /topic_bridge/status r1_interfaces/msg/ManagerStatus
ros2 topic echo /control_server/status r1_interfaces/msg/ManagerStatus
```

## Parameters and command freshness

All parameters are in `config/topic_bridge.yaml`. Launch arguments override YAML
values; a custom configuration uses `config_file:=/absolute/path/config.yaml`.

| Parameter | Default | Contract |
| --- | --- | --- |
| `topic_name` | `/joy` | Upstream ROS topic; sensor-data best-effort and reliable publishers both match |
| `msg_type` / `csm_mode` | `joy` / `topic` | `joy`, `twist`, `string`; String is topic-only |
| `server_name` | `control_server` | R1 target manager name |
| `csm_name` / `master_name` | `topic_bridge` / `csm_master` | This manager and the shared master |
| `controller_name` | `local_xbox` | System-wide unique identity; empty YAML string falls back to channel name |
| `channel_name` | `topic_bridge_control` | Unique data topic/service name |
| `priority` | 80 | Generic R1 priority 1–100, interpreted by the consumer |
| `timeout_ms` | 2000 | Time without data before TIMEOUT; 0 disables it in topic mode |
| `disconnect_timeout_ms` | 10000 | Time since last data before removal; 0 disables removal; otherwise greater than timeout |
| `csm_status_timer_interval_ms` | 100 | Manager tick, 1–299 ms for the configured 600 ms heartbeat timeout |
| `input_timeout_ms` | 250 | Maximum local receipt age immediately before issuing a send |
| `registration_timeout_ms` | 1000 | Bounded synchronous registration wait, 1–5000 ms |
| `registration_retry_ms` | 200 | Initial/new-intent retry interval while input is fresh, 1–60000 ms |

Both inactivity thresholds measure from the last activity. INITIAL has not seen
any data and transitions directly to DISCONNECTED if its removal threshold
expires. R1 removes the legacy LOW_FREQ state, frequency/keep-alive descriptor
fields and `controller_priority_type`. Priority 94 has no reserved transport
meaning; use the server's documented arbitration policy.

For additional bridges, set **both** a unique controller and channel, including
Twist/String bridges; they otherwise inherit the joystick identity:

```bash
ros2 launch rv2_csm_topic_bridge topic_bridge.launch.py \
  topic_name:=/cmd_vel msg_type:=twist controller_name:=navigation \
  channel_name:=navigation_control csm_name:=navigation_bridge priority:=20
```

The mailbox keeps only the latest pending sample. Blocking registration/service
calls may coalesce intermediate inputs. Receipt time uses a monotonic clock;
message header timestamps do not authorize stale replay. The worker refreshes
the mailbox after registration, drops old samples, and never periodically
re-sends cached commands. A timed-out service request is not retried because its
outcome is unknown. A service call already sent can finish during shutdown;
shutdown waits for that configured timeout. Select `timeout_ms` accordingly.

Topic registration readiness does not guarantee DDS subscriber discovery has
finished; a first one-shot sample may be lost. This is a live control stream, so
publish continuously with an appropriate driver autorepeat rate. Parameter
changes require restarting the bridge. Invalid parameters fail construction.

## Docker tests

Initialize the package-owned framework submodule and keep sibling R1 transport
and interface checkouts available as listed in `test_depends.repos`:

```bash
git submodule update --init --recursive
./r1_test_framework/test_build.sh
./r1_test_framework/test_deps.sh
./r1_test_framework/test_run.sh
./r1_test_framework/test_lint.sh
./r1_test_framework/test_clean.sh
```

The framework uses the official Jazzy base image, installs/builds/tests only in
Docker and stores artifacts under ignored `test_env/jazzy/`. Run ROS test jobs
serially across packages; default Docker bridge networking does not isolate DDS.
The no-argument run executes normal CTest; this owner has no defined sanitizer
profiles, so ASan/UBSan/TSan are explicitly N/A, not sanitizer coverage claims.

Use `test_run.sh -s unit` or `-s integration` for focused checks. Unit tests cover
configuration conversion/validation, priority and integer bounds, heartbeat
policy, mailbox replacement/wakeup/shutdown and freshness boundaries.
Integration tests exercise all five transport bindings with exact payload
assertions, best-effort Joy, timeout recovery without cached replay, physical
reconnection, late server startup, delayed registration stale-input rejection,
real-master server restart and bounded shutdown during registration/service
waits. These automated publishers do not replace the physical joystick check.
