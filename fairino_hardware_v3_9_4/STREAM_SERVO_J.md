# Buffered ServoJ streaming

Use the `stream_servo_j` action for trajectories that have already been resampled to a
fixed joint-space period. The complete flattened trajectory crosses ROS once; the driver
then sends it from a local worker using `ServoMoveStart(1)`, repeated UDP
`ServoJ(..., period_sec, ..., command_id, 1)`, and `ServoMoveEnd(1)`.

The legacy `fairino_remote_command_service` forms `ServoMoveStart(...)`, `ServoJ(...)`,
and `ServoMoveEnd(...)` remain available for compatibility. They are not recommended for
sample-by-sample trajectory streaming.

Example goal with two six-joint samples at 4 ms:

```bash
ros2 action send_goal /stream_servo_j fairino_msgs/action/StreamServoJ \
  "{joints_deg: [0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0], period_sec: 0.004, communication_type: 1}" \
  --feedback
```

Goals are rejected unless they contain a non-empty multiple of six finite joint values,
use UDP communication type `1`, use a supported period, and change no joint by more than
the configured per-command limit between adjacent samples. Only one buffered stream can
be active. Other motion commands are rejected until it finishes; action cancellation or
the legacy `StopMotion()` command preempts the stream.

The result reports requested and sent sample counts, the controller error, final command
ID, maximum measured SDK-command interval, missed deadlines, and cancellation state.
Feedback is throttled independently of the command loop (50 ms by default).

Configuration parameters and defaults:

- `servo_j.minimum_period_sec`: `0.001`
- `servo_j.maximum_period_sec`: `0.016`
- `servo_j.maximum_joint_step_deg`: `1.0`
- `servo_j.deadline_tolerance_sec`: `0.00025`
- `servo_j.maximum_lateness_sec`: `0.008` (`0.0` falls back to one requested period)
- `servo_j.feedback_period_sec`: `0.05`

## Timing: miss tolerance versus fatal lateness

Each sample has an absolute deadline `start + index * period_sec`. Two thresholds apply:

- `servo_j.deadline_tolerance_sec` only counts a missed deadline in the metrics
  (`missed_deadlines` in the result). It never aborts the stream.
- `servo_j.maximum_lateness_sec` is the fatal budget. If the loop wakes later than this
  after a deadline, or the SDK call completes later than this after the deadline, the
  stream aborts with `StopMotion()` + `ServoMoveEnd()` and the action result reports
  failure.

The driver host is best-effort: Docker plus the stock CFS scheduler is not a motion
real-time environment, and multi-millisecond scheduling stalls are normal under load.
The fatal default is therefore `0.008` (two 4 ms periods); the controller's own
buffering absorbs occasional late commands of that size. Setting
`servo_j.maximum_lateness_sec` to `0.0` restores the strict one-period budget, which is
only realistic with RT scheduling. Do not disable the check with a huge value: a stream
that has genuinely stalled must still stop the robot.

## Abort diagnostics

When a stream aborts on a fatal lateness, the result `message` keeps its existing
prefix (`ServoJ timing failure: command deadline exceeded maximum lateness` or
`ServoJ timing failure: SDK command completed after maximum lateness`) and appends a
bracketed breakdown, for example:

```text
ServoJ timing failure: command deadline exceeded maximum lateness [sample_index=812
samples_sent=812 period_sec=0.004000 wake_lateness_ms=9.412 sdk_call_ms=0.000
lock_wait_ms=0.000 prev_sdk_call_ms=7.913 prev_lock_wait_ms=0.004 max_interval_ms=7.951
missed_deadlines=41]
```

- `wake_lateness_ms`: how late the loop woke relative to the sample's absolute deadline.
- `sdk_call_ms` / `lock_wait_ms`: SDK/UDP call duration and time spent waiting for the
  shared SDK mutex for the sample that tripped the post-call check (`0.000` when the
  abort happened before this sample's `ServoJ()` was issued).
- `prev_sdk_call_ms` / `prev_lock_wait_ms` (pre-call aborts only): the same split for
  the previous sample, which is what pushed the wake past the deadline.
- `max_interval_ms` / `missed_deadlines`: running metrics up to the abort.

The same message is logged at ERROR by the command server, so
`docker compose logs fr5-driver | grep 'ServoJ timing'` shows whether a stall came from
OS scheduling (large `wake_lateness_ms` with small SDK numbers), SDK/UDP time
(`sdk_call_ms`), or mutex contention (`lock_wait_ms`).

## State polling during a stream

The existing `nonrt_state_data` timer runs at 50 ms (20 Hz) outside a stream. Its SDK poll
is suppressed while a buffered stream is active because `GetRobotRealTimeState()` and
`ServoJ()` share the same SDK mutex; allowing the poll during a stream can block a command
past its deadline. In addition the poll only uses `try_lock` on the SDK mutex and skips
the sample when the mutex is busy, so it can never queue behind (and then stall) a
streaming command. Consumers should expect `nonrt_state_data` (and anything derived from
it, such as TF) to pause for the duration of a buffered stream and resume on the first
timer callback after the stream. Service and action callbacks use separate callback
groups, while access to the shared FAIRINO SDK object is serialized. The blocking stream
itself does not run on an executor thread.

Clients that enforce a final joint tolerance should include a short run of repeated final
samples in the submitted buffer, then wait for a `nonrt_state_data` message published
after the action completes before checking the endpoint. Checking the cached state
immediately can report the controller's normal following lag (or the last pre-stream
state) as a goal-tolerance failure.
