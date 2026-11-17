# Offline revalidation, 17 November 2026

The requested timed-loop instrumentation and bounded/latest-value action feedback
handoff are already implemented by `4bdf0c35` and `6ff31b4a`, retained at tested
head `edca0326a694eb290a5148e6ae0042291e0a8931`. No duplicate implementation or
production configuration change was added during this continuation.

Fresh validation passed:

- Seven standalone CTest executables, including real-library ServoJ serialization
  for pinned SDK 2.4.1 and SDK 2.3.4 (`5925704ec25b27b3787544632a66a3702d80866a`).
- Full ROS 2 Jazzy SDK 2.4.1 build and 53 cases in six ServoJ targets, FastDDS.
- Full SDK 2.3.4 diagnostic build and 57 cases in seven targets, using the existing
  private investigation's `office-qualification-20261106/sdk234-diagnostic.patch`
  in a scratch source copy. This selects old library names, preserves legacy
  ServoMoveStart communication-type-only semantics and rejects unsupported tuning.
- Twelve GRIP command/fake-driver integration cases covering stale/fault stopping,
  strict timestamps, terminal results/server survival, large handoff stalls and
  buffered timing. Forty-seven unrelated cases were deselected.

The publisher tests hold an actual RMW feedback publication for at least 500 ms.
The sender, action cancellation, terminal results and next session still complete.
The separate SDK-stall tests model 325.090 ms, including a late final send; they
retain deadline failure and StopMotion-before-ServoMoveEnd. Immutable UUIDs and
coalescing prevent pending progress being relabeled as the next session.

Merely replacing the vendor directory with SDK 2.3.4 **failed** the full build:
the current three-argument start API and linked library name target SDK 2.4.1.
The corrected diagnostic build is a separate variant, not proof that branch-head
SDK 2.4.1 supports an older controller. Package name does not identify SDK version.
Both initial failure and adapted success are retained in the private GRIP
investigation, `workstation-20261117/validation/`, with GoogleTest XML and hashes.

Builds used the cached image `grip-fr5-driver:jazzy`, image ID
`sha256:12ad5662442039303affe39c152ca618dfca10ba137fa590f03483e57de891d8`.
Containers had no network, no capabilities, a read-only root/source mount and
writable scratch. No real robot/gripper service was started and nothing deployed.
The separate operator-authorized receive-only readiness check is documented in
the private investigation; it does not qualify either SDK's motion behavior.

Remaining limits: existing compiler warnings were not addressed; Zenoh and
sanitizers were not rerun. Existing buffer/SDK locks remain. A blocked SDK call
cannot be interrupted by this handoff and is timed after return. An indefinitely
blocked middleware publication can delay process shutdown. An already-entered
publication may complete after close on its original goal; terminal counts belong
to the result. No physical timing bound or old-controller compatibility follows
from these builds and mocks.
