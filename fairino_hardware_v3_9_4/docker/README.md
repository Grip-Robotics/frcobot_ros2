# fairino_hardware_v3_9_4 — Docker

## Owner / maintainer

Grip Robotics — robot integration / manipulation stack.

## Base image

`grip-ros-zenoh:jazzy` (ROS 2 Jazzy + `rmw_zenoh_cpp`). Build it from the grip repo before this service:

```bash
docker compose --profile build build ros-zenoh-base
```

## Package

| Field | Value |
|-------|-------|
| ROS package | `fairino_hardware_v3_9_4` |
| Default executable | `ros2_cmd_server` |
| Default run command | `ros2 run fairino_hardware_v3_9_4 ros2_cmd_server` |

## Dependencies

### Same-repo siblings (copied in Dockerfile)

- `fairino_msgs` — message/service definitions required at build time.

### External git dependencies (`deps.repos`)

None. All required sources live in this repository.

### Non-rosdep system dependencies

- **Fairino vendor SDK** (`libfairino.so.2.3.4`) is bundled under `libfairino/lib/` and installed next to the node binaries via `$ORIGIN` RPATH. No separate apt package is required.

## Build context

The Dockerfile assumes the **grip repository root** as build context (after `frcobot_ros2` is imported via `import-deps.sh`):

```bash
# From grip repo root, after ros-zenoh-base is built:
docker compose build fairino-cmd-server
```

### Subrepo-only development

When building from this repository in isolation, mirror the grip layout or override the Dockerfile `COPY` paths. The local copy of `docker/ros_entrypoint.sh` is provided for standalone testing; in grip, the platform copies `docker/ros_entrypoint.sh` from the repo root (same content).

## Runtime environment (set by grip `compose.yaml`)

This image does **not** start a Zenoh router. Connect as a Zenoh client to the shared `zenoh-router` service:

| Variable | Example |
|----------|---------|
| `RMW_IMPLEMENTATION` | `rmw_zenoh_cpp` |
| `ROS_DOMAIN_ID` | shared stack ID |
| `ZENOH_ROUTER_CHECK_ATTEMPTS` | `0` |
| `ZENOH_CONFIG_OVERRIDE` | `mode="client";connect/endpoints=["tcp/zenoh-router:7447"]` |

## Hardware / network

- **Robot controller IP**: default `192.168.57.2` (compile-time define in `data_type_def.h`). Override at runtime via grip compose environment or rebuild if your subnet differs.
- **Network**: the container must reach the Fairino controller on the robot LAN (typically `192.168.57.0/24` or `192.168.58.0/24`). Attach the service to a Docker network that routes to the robot, or use `network_mode: host` if required by your deployment.
- **GPU / USB**: not required for this package.

## Test mode

Interactive shell without auto-starting the node:

```bash
docker compose run --rm fairino-cmd-server bash
```

Inside the shell, source is already applied by the entrypoint when you run a command. For a manual check:

```bash
ros2 node list   # from another Zenoh-connected container
```

## Suggested grip `compose.yaml` service

```yaml
fairino-cmd-server:
  build:
    context: .
    dockerfile: src/frcobot_ros2/fairino_hardware_v3_9_4/docker/Dockerfile
  profiles: [fairino, stack-vision-fairino]
  networks: [grip-net]
  environment: *ros-env
  depends_on: [zenoh-router]
  command: ["ros2", "run", "fairino_hardware_v3_9_4", "ros2_cmd_server"]
  # extra_hosts or network_mode: host if the robot is on a non-routable LAN
```
