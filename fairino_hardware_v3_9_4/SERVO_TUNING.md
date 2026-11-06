# ServoMoveStart filter and gain settings

The command server uses SDK 2.4.1's `ServoMoveStart(comType, filtertime, gains)`.
Controller software 4.0.1 is the vendor compatibility target; controller acceptance
and physical response must be verified separately. This is not a measured lag fix.

Two startup-only ROS parameters apply to both the buffered `StreamServoJ` action
and the legacy `ServoMoveStart()` / `ServoMoveStart(comType)` command service:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `servo_j.filter_time_ms` | `0` | Integer filtering time in **milliseconds** |
| `servo_j.joint_gains` | `[-1.0, -1.0, -1.0, -1.0, -1.0, -1.0]` | Six position-loop gains in J1 through J6 order; -1 requests the controller default |

The shipped `config/servo_start.yaml` contains vendor defaults, not tuned values.
Load an edited copy with `--ros-args --params-file /path/to/servo_start.yaml`, or
append the following arguments to the existing driver command:

```sh
-p servo_j.filter_time_ms:=0 \
-p 'servo_j.joint_gains:=[-1.0, -1.0, -1.0, -1.0, -1.0, -1.0]'
```

Use floating-point YAML literals for the gain array (`-1.0`, not `-1`). Each robot
process can load its own parameter file. The parameters are read-only after node
startup, so a parameter update cannot change an active servo session. Restarting
a live driver or upgrading firmware is a separate deployment step.

Invalid filter times, gain arrays other than length six, NaN/Inf, unsupported
negative values, and values that would overflow the vendor's UDP formatting
buffer are rejected before connecting. The driver does not clamp requested values.
Vendor/model-specific tuning ranges still need confirmation; these checks only
validate the API representation. The SDK formats UDP gains to three decimal places.

The legacy text command accepts only its original optional communication-type
argument; trailing arguments are rejected. Tuning comes from the same startup
parameters as the action, so the two paths cannot silently use different settings.
The shared SDK adapter preserves both XML-RPC (0) and UDP (1) and forwards errors.

The `ServoJ()` sample `filterT` and `gain` arguments remain zero because the vendor
still marks those separate arguments unavailable. Neither host sample timestamps
nor SDK `cmdT` is changed by this feature. Defaults in the newer SDK are not proof
that an older controller's previous internal filtering is reproduced.

Startup logs and each stream-result trace record the **requested** filter time and
gain vector (`servo_start_filter_time_ms`, `servo_start_joint_gains`). A successful
UDP send is not a controller acknowledgment. Use the existing UDP reply diagnostics
and controller feedback to verify acceptance before interpreting the lag result.

SDK provenance and checksums: [libfairino/UPSTREAM.md](libfairino/UPSTREAM.md).

## Offline validation

The functional tests cover argument order, both transports, error propagation,
invalid inputs, settings captured at startup, trace fields, and unchanged 5 ms
sample cadence. An additional test links the real SDK and intercepts its UDP send
boundary to check the serialized filter/gain payload without a robot connection. Existing streamer cancellation and deadline tests remain in place.
Build and run them with the normal ROS 2 Jazzy colcon workflow. No robot connection
is needed by the tests.
