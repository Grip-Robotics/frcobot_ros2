# frcobot_ros2

Branch `fix/trajectories_delay` preserves two development states for ServoJ timing investigation:

- Commit `5925704ec25b27b3787544632a66a3702d80866a` retains SDK 2.3.4 and adds opt-in successful-stream trace recording. This is the source variant used for the recorded timing experiments.
- The branch head adds SDK 2.4.1.0 and startup filter/six-joint gain forwarding. Controller 4.0.1 is its vendor compatibility target; it is an offline-tested candidate, not a demonstrated physical lag fix or an automatic controller upgrade.

Read [ServoMoveStart tuning support](fairino_hardware_v3_9_4/SERVO_TUNING.md),
[SDK provenance](fairino_hardware_v3_9_4/libfairino/UPSTREAM.md) and
[stream timing/diagnostics](fairino_hardware_v3_9_4/STREAM_SERVO_J.md).
`servo_j.trace_successful_streams` defaults to false; when enabled, bounded sample traces are written after a stream completes.
Host scheduling and SDK `cmdT` remain independent through the existing override; this branch does not select a new timing correction.
The package name `fairino_hardware_v3_9_4` is retained for compatibility and does not identify the SDK bundled at the branch head.

This is the ROS2 API project of Fairino robot(sofeware version must greater than V3.7.1), a serial of functions which based on Fair SDK API but were simplified are created, user can call them through service message.
API_description.md list all API functions.
Tutorial of installing and uasage of ROS2 API, please refer to the Fair document platform:https://fair-documentation.readthedocs.io/en/latest/ROSGuide/index.html#frcobot-ros2.

Version histroy:
2023.7.18 V1.0

For timing-sensitive ServoJ trajectories on the 3.9.4 driver, use the buffered
`fairino_msgs/action/StreamServoJ` interface described in
[`fairino_hardware_v3_9_4/STREAM_SERVO_J.md`](fairino_hardware_v3_9_4/STREAM_SERVO_J.md).
The string-based remote command service remains available for compatibility.
