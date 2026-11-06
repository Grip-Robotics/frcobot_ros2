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
Feedback is throttled independently of the command loop (50 ms by default) and
published by a separate best-effort worker through a bounded latest-value handoff.

Configuration parameters and defaults:

- `servo_j.minimum_period_sec`: `0.001`
- `servo_j.maximum_period_sec`: `0.016`
- `servo_j.maximum_joint_step_deg`: `1.0`
- `servo_j.deadline_tolerance_sec`: `0.00025`
- `servo_j.maximum_lateness_sec`: `0.008` (`0.0` falls back to one requested period)
- `servo_j.feedback_period_sec`: `0.05`
- `servo_j.sdk_cmdt_override_sec`: `0.0` (startup-only experimental override)

## Timing: miss tolerance versus fatal lateness

The SDK override is read-only after node startup. Zero preserves the existing
behavior: the action's host period is also passed as SDK `cmdT`. A positive finite
value must lie within the configured minimum/maximum period; it is frozen when a
stream is reserved and remains unchanged through tail revisions. It changes only
the SDK argument. Host deadlines, sample indexes, feedback phase, and all existing
joint-step and lateness checks still use the action's `period_sec`.

This is an opt-in diagnostic facility, not a validated timing correction. For a
reviewed stationary A/B test, an override of `0.006` permits comparing action host
periods `0.006` and `0.006006` while wire `cmdT` stays `0.006000`. A moving caller
must resample the original trajectory at the selected host period; sending old
samples more slowly changes the trajectory timing. Firmware `cmdT` resolution and
underflow behavior remain unverified. A flat queue during a hold does not prove
correct physical motion or belt-contact timing.

Stream result traces record `period_sec` (host period), `sdk_cmdt_sec`, the actual
float argument as `sdk_cmdt_float_sec`, and `sdk_cmdt_override_active`. The legacy
remote command service is unaffected by this action-specific setting.

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
  the previous sample. These are context; they do not establish why the wake was late.
- `max_interval_ms` / `missed_deadlines`: running metrics up to the abort.

The same message is logged at ERROR by the command server. A large wake lateness
with small SDK numbers leaves sleep/scheduling and other intervening stages open;
inspect the stage trace below before attributing a cause.

## State polling during a stream

The existing `nonrt_state_data` timer runs at 50 ms (20 Hz), including during a stream.
It uses `try_lock` on the shared SDK mutex and skips the sample when the mutex is busy.
Service and action callbacks use separate callback groups; the blocking stream itself
does not run on an executor thread. The SDK poll reads its cached controller state.
With automatic reconnect enabled, the pinned SDK 2.3.4 can return success and an
all-zero cache despite receiving no feedback. A successful getter alone is therefore
insufficient for publication.

The first nonzero frame-counter/controller-time identity primes a progress check;
publication begins only after that identity changes. Repeated cached identities and
an all-zero identity are not published or assigned fresh ROS timestamps. Counter
wrap and a changed controller clock at the same counter are accepted. The existing
GRIP 500 ms missing-state watchdog can then detect a stopped stream. SDK-error
messages retain their existing publication path. The state callback group is
mutually exclusive; this check adds no sender work, SDK calls or locking.

This is an observation-of-progress check, not proof of wire age or clock alignment.
SDK 2.4.1 uses its CNDE-backed cache, so the check uses common counter/time fields
without assuming that raw TCP header/length fields have been populated. Field
availability still requires live qualification for each firmware/SDK pairing.

Clients that enforce a final joint tolerance should include a short run of repeated final
samples in the submitted buffer, then wait for a `nonrt_state_data` message published
after the action completes before checking the endpoint. Checking the cached state
immediately can report the controller's normal following lag (or the last pre-stream
state) as a goal-tolerance failure.

## Bounded fault capture

Set these driver parameters at startup to capture the next fault:

```yaml
servo_j.trace_directory: /workspace/grip/logs/servo
servo_j.trace_samples: 8192
```

An empty directory parameter (the default) disables capture. Each ROS namespace owns a
subdirectory with punctuation replaced by underscores, for example `_robot_1`. Mount the
base directory on the host to preserve files when recreating a container. The sample
count is clamped to 1–32768 when enabled. At 250 Hz, 8192 records retain approximately
32.8 seconds. This capture does not change motion limits, pacing, cancellation, or fault
handling and does not issue extra SDK state queries.

The stream worker copies a fixed-size record into preallocated memory for each SDK
attempt. It also records a sample rejected by the lateness check before an SDK attempt.
It does no trace formatting, logging, allocation, or file I/O in the send loop.
When capture is enabled, additional monotonic reads bracket the stages described
below. Formatting and file writes happen after termination:

| File | Contents and retention |
| --- | --- |
| `stream-fault-0.jsonl` through `stream-fault-3.jsonl` | Last four failed/cancelled streams; one result header followed by the retained sample records, oldest first. Slots are reused after process restart. |
| `stream-events.jsonl` and `.1` | One result header per stream, including successful streams; each file is capped at 1 MiB. |
| `controller-state.jsonl` and `.1` | Existing 20 Hz SDK state reads, suppressed cached samples, skipped polls, and errors; each file is capped at 8 MiB. Writes occur after releasing the SDK mutex and any state publication. |

Fault files are written to `.tmp` then renamed. Copy them promptly when preserving an
incident because later faults rotate the slots. A process crash or forced kill before
termination cannot dump the in-memory send ring; already written controller rows remain.
Write failures warn without changing robot control. There is no full trajectory archive:
the send ring preserves exact attempted joint samples across accepted tail revisions.

All rows are JSON with schema version 2 in stream-result headers and version 1 in
controller headers. Existing fields retain their meanings. Nonfinite state
values are `null`. The stream result header identifies the ROS action `goal_id`, period,
first successful send's `stream_start_ros_ns`, monotonic deadline origin, terminal
status, sent/requested counts, and number of overwritten ring records. `servo_sample`
rows contain:

- `sample_index`: global zero-based index, unchanged when an old prefix is retired.
- `buffer_revision`: revision active when this sample was copied from the buffer;
  preserved prefix samples can carry a newer revision without changing their values.
- `command_id`, `sdk_attempted`, `sdk_error`: distinguish an accepted SDK call from a
  failed attempt or a late sample never submitted (`command_id=-1`). SDK success proves
  host submission, not controller receipt or physical tracking.
- `joints_deg`: the exact six values supplied to `ServoJ` for an attempted sample.
- `deadline_mono_ns`, `wake_mono_ns`, `return_mono_ns`, `lock_wait_ns`, `sdk_call_ns`:
  pacing and SDK timings. `return_mono_ns` is zero for an unattempted late sample.
- `loop_begin_mono_ns`, `buffer_ready_mono_ns`: buffer-lock acquisition and sample-copy envelope.
- `sleep_enter_mono_ns`, `wake_mono_ns`: absolute-sleep/scheduler envelope.
- `sdk_enter_mono_ns`, `sdk_return_mono_ns`: complete robot adapter call, including
  argument preparation, SDK mutex wait, diagnostics and vendor serialization/send.
  Existing `lock_wait_ns` / `sdk_call_ns` split the inner mutex and SDK intervals.
- `sdk_return_mono_ns` to `return_mono_ns`: includes the first-send ROS clock read
  on sample zero and ordinary progress bookkeeping.
- `feedback_handoff_begin_mono_ns`, `feedback_handoff_end_mono_ns`: bounded progress
  copy, or both zero if no progress update was due.
- `loop_end_mono_ns`: immediately before copying the trace record into its ring.
  Inspect adjacent rows to locate gaps between iterations; zero means a stage was
  not entered. These intervals include preemption and do not alone identify a blocking function.

Controller rows include host ROS/monotonic read times, the SDK's raw calendar
`controller_time` array `[year, month, day, hour, minute, second, millisecond]`, frame
counter, vendor command counter, queue length, motion/error/safety flags, and joint
position/velocity/acceleration and torque. `queued_target_deg` is `lastServoTarget`, the
newest queued target; it is not an instantaneous target aligned with measured joints.
The SDK exposes no joint target velocity or saturation flag, so those availability
fields are explicitly false. `servo_j_cmd_num` is a controller counter; do not assume
it equals the host `command_id` without establishing its offset/reset behavior.

`observed_sample_unchanged_ns` measures time since the observed controller time/frame
identity last changed. It can expose repeated cached samples and grows across failed
or skipped reads; it is not an absolute network packet age. A skipped SDK mutex poll
sets `sdk_read_skipped=true`; its `sdk_error=-1` is a local sentinel.
`state_published=false` identifies skipped polls and suppressed cache observations;
`frame_head` is diagnostic only and is not required by the progress check.
Controller clock synchronization with host ROS time is not assumed. Join state and send records using
their host monotonic timestamps, then inspect controller time/frame progression.

The moving-tail implementation and `ReplaceServoJTail.srv` were recovered unchanged
from the deployed driver image
`sha256:170e3868e8b71960b8c2f7acb22613d073d4a6e462ab09518dbe29ebb2d92972`
before adding this instrumentation. Tail replacement validates identity, revision,
finite samples, step limits, and the future splice guard; diagnostics do not alter it.

### UDP reply diagnostics

The same nonempty `servo_j.trace_directory` also installs SDK 2.3.4's
`SetCmdRpyCallback`. This is a local callback registration, not another controller
query. The SDK calls it on its UDP receiver thread. The callback copies at most 64
content bytes plus source, envelope counter, command number, and host monotonic receive
time into a fixed queue. It does no logging, file I/O, DDS publication, locking, or
allocation of its own (the SDK callback signature itself passes a `std::string` by
value). A separate fixed queue captures each start/ServoJ/end call before entering the
SDK. The existing 20 Hz trace writer drains both queues outside the SDK mutex.

Result and controller-state headers contain `udp_replies`. Nested
`capture.tracking` records the session number, start request/reply times and latency,
start envelope counter, sent call count, matched reply count, last inferred replied
ordinal/command ID, latest/maximum reply latency, end reply time, and unmatched,
ambiguous, malformed, and out-of-order counts. Ordinals start at **1** for the first
ServoJ call in a session. `calls_without_matched_reply` includes losses and ambiguous
matches; it is not a measured controller queue length. An absent start reply has a
null latency and frame counter -1. A nonzero `callback_registration_error` or
`capture.enabled=false` means the callback capture is unavailable.

Fault files append up to 8192 `servo_udp_reply` records, including raw content and
receive time, after the send samples. Reply records identify commands 689 (start),
376 (ServoJ), and 690 (end). Matching predicts the eight-bit envelope counter from
the observed start reply and per-session call ordinal. Multiple outstanding candidates
after counter wrap remain ambiguous. New starts discard old pending candidates and
report `previous_session_unreplied_calls`; receive timestamps before that start are
rejected. A missing prior start reply prevents choosing a new start anchor. Send
failure, queue loss, or overwriting an unanswered call invalidates mapping for that
session. Each queue holds 2047 usable records; overflow drops diagnostics and reports
it instead of delaying motion. The pending window holds 4096 calls.

Correlation is explicitly **inferred**: UDP duplicates or very late replies carrying
a reused counter cannot always be distinguished across sessions or full wraps. Neither
raw reply content `"1"` nor `last_replied_ordinal` is asserted to mean successful
physical completion. Callback receive time includes SDK receiver scheduling and is
not a kernel wire-arrival timestamp. Successful SDK UDP `ServoJ` returns after its
`sendto` call, without waiting for a controller reply. These diagnostics add no
handshake wait, throttle, timeout policy, resampling, or motion-result changes.

Replies arriving after cancellation can update subsequent controller-state summaries;
the already-written fault file is a snapshot taken at stream termination. Preserve
both files when diagnosing late replies. Because the SDK callback has no context
pointer and its receive thread is detached, its small process-owned mailbox remains
allocated until process exit. Shutdown atomically disables it before SDK destruction;
late callbacks are ignored, and the mailbox cannot be re-enabled in that process.
One SDK receiver per command-server process is required, as in the existing deployment.

Offline validation is `test_servo_j_udp_diagnostics` alongside
`test_servo_j_streamer`; it covers wrap, loss, reordering, duplicate/stale replies,
session boundaries, send failure, bounded raw content, queue overflow, concurrent
queue ordering, and callbacks after shutdown. No controller is needed.

## Action-feedback isolation and result boundaries

The node creates one publication worker before applying the sender's SCHED_FIFO
policy. Each goal gets its own fixed progress slot and immutable callback/goal
identity. The sender makes one atomic-flag attempt to copy three scalar values;
it never waits for that slot, allocates a ROS message, wakes a condition variable,
or calls DDS. Pending progress is coalesced, and slot contention drops a diagnostic
update. The worker polls at 1 ms and releases all handoff locks before allocation
or publication. There is no unbounded feedback queue and no per-goal worker join.

Closing a session prevents a new publication claim and discards pending progress.
An already claimed call can complete later, tied to its original goal UUID. A
stalled old publication can delay progress for later goals, but cannot relabel it.
Sending, cleanup and the terminal result path do not wait for this worker. The
worker publishes the generated action feedback message directly on the resolved
`/_action/feedback` topic, with the default action-server QoS and original goal
UUID. Calling `ServerGoalHandle::publish_feedback` instead held a shared action
server lock during a deliberately stalled middleware call, blocking result
delivery in both tested RMWs. The separate typed publisher avoids that lock while
preserving the action wire format, namespace and action-name remapping.
The result remains the authoritative final sample count; a terminal progress
message is not guaranteed. Other shared ROS/DDS resources can still stall.
Publisher exceptions are counted and contained. The node joins the publication
worker only on shutdown; permanently blocked middleware can therefore delay shutdown.
Detaching a worker that retains node/action resources would be unsafe.

Stream-result traces include an `action_feedback` snapshot taken at close:
published/coalesced/contended/exception counts, latest publication begin/end
monotonic timestamps, maximum completed publication duration, and
`in_flight_at_close`. The snapshot does not update if middleware returns later.
This does not remove existing SDK or trajectory-buffer locks, nor allocations
inside the vendor SDK. Those remain measured boundaries. Cancellation, SDK
errors, fatal lateness, tail revisions, and action terminal-state selection are
unchanged. GRIP's controller-fault/stale-state watchdogs retain their independent
state subscription and cancellation path.

The mock tests deliberately block publication through completion, another
session, cancellation and SDK failure, then separately block SDK sending and
verify deadline abort plus StopMotion-before-ServoMoveEnd. They also check latest
progress, publication exceptions, ROS-clock versus SDK timing, and bounded trace
retention. They establish software behavior with mocks, not Linux real-time bounds.

`test_servo_j_action_feedback` adds a real Jazzy action server/client with a fake
robot and virtual sender clock. It interposes `rmw_publish` to block the feedback
topic for at least 500 ms. Successful and canceled results, another goal, and
correctly attributed progress after release must still arrive. A separate SDK
error and 325.090 ms final-send stall must deliver aborted results and retain
StopMotion-before-ServoMoveEnd. The server uses intra-process communication and a
remapped action in a namespace; the client crosses the real RMW boundary. This
test does not link the vendor SDK or start the hardware node.

On 6 November 2026, a cached Jazzy image built the full driver and passed all 53
cases in six ServoJ CTest targets, including ten repeats of the action test with
FastDDS. Five action-test repeats also passed with Zenoh. Replacing only the
callback with `ServerGoalHandle::publish_feedback` made the result-delivery check
fail on both RMWs. Twelve selected GRIP command/fake-driver integration cases
also passed, covering fault/stale feedback, cancellation, invalid source times,
handoff stalls and period preservation. These container tests used no external
network or robot; they do not prove the cause of the historical host pause.

Without ROS installed, run the same C++ tests with local GoogleTest sources:

```sh
cmake -S fairino_hardware_v3_9_4/test/offline -B work/offline-tests \
  -DGOOGLETEST_SOURCE_DIR=/path/to/local/googletest
cmake --build work/offline-tests -j4
ctest --test-dir work/offline-tests --output-on-failure
```

No dependencies are downloaded. Optionally set `SDK234_PACKAGE_ROOT` to the
`fairino_hardware_v3_9_4` directory extracted from commit
`5925704ec25b27b3787544632a66a3702d80866a` to test ServoJ serialization against that
SDK as well. SDK tests interpose the UDP SendFrame boundary and abort before any
unexpected connect/send/sendto call. SDK 2.3.4 uses a different constructor and
communication-type-only ServoMoveStart; branch head still requires SDK 2.4.1.0's
filter/six-gain start interface. These tests do not make the branch head compatible
with older controllers. The Jazzy build and mock action-client checks above do not
replace fresh controller identity, SDK compatibility review and authorized live
qualification before deployment. No firmware or runtime image is changed here.
