# Vendored FAIRINO C++ SDK

This package vendors the Linux x86-64 headers and unmodified shared library from:

- Repository: https://github.com/FAIR-INNOVATION/fairino-cpp-sdk
- Tag: `v2.4.1-robot4.0.1`
- Commit: `2c849cc672d633a814bac2dbe3f8e94d31d756ef`
- Header source: `linux/libfairino/include/{robot.h,robot_types.h,robot_error.h}`
- Library source: `linux/libfairino/lib/x86.zip`, member `lib/libfairino.so.2.4.1`
- Library SHA-256: `90cf52b90a577e5c236e96a75da98b7b098767c2aa6cbf6795498d742e9494c1`
- License: Apache-2.0, copied alongside this file.

The SDK identifies itself as `SDK V2.4.1.0-robot v4.0.1`. This is the SDK's
compatibility target, not an observed controller version. Verify the robot's
software version before deploying this candidate. A controller firmware upgrade
is separate from rebuilding this driver.

The existing `fairino_hardware_v3_9_4` package name is retained to avoid changing
launch files and downstream package references. Its name no longer identifies
the SDK revision; the vendor files, link target, and startup compatibility banner
do. Driver banner version is 3.0.1; the ROS message schema remains 3.0.6.

Header whitespace is normalized to LF with trailing spaces removed and one
space-before-tab indentation corrected. Declarations and comments otherwise
match the pinned upstream headers.
