# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this package is

`mujoco_sim_ros2` is an `ament_cmake` ROS2 package that wraps MuJoCo's upstream `simulate` application and adds a `pluginlib`-based extension point so the simulation can be driven by ROS2 / `ros2_control`. It builds a single executable, `mujoco_sim`. Used together with [`mujoco_ros2_control`](https://github.com/rxdu/mujoco_ros2_control) it gives a `gz_ros2_control`-like setup for MuJoCo.

Supported/tested target: **Ubuntu 24.04, ROS2 Jazzy, MuJoCo 3.9.0**. The package version tracks the MuJoCo version it syncs against (currently `3.9.0`).

## Vendored vs. custom code — read this before editing `src/`

Most of `src/` is copied **verbatim** from MuJoCo's `simulate` app and should be treated as upstream/third-party. Do not refactor or restyle it; changes here are only ever for syncing with upstream.

- **Vendored unchanged:** `simulate.cc/.h`, `glfw_adapter.*`, `glfw_dispatch.*`, `glfw_corevideo.*`, `platform_ui_adapter.*`, `macos_gui.mm`, `array_safety.h`, `src/README.md`.
- **Custom ROS2 code (this is where intended changes go):**
  - `include/mujoco_sim_ros2/mujoco_physics_plugin.hpp` — the plugin base class / `pluginlib` interface.
  - `src/main.cc` — modified upstream `main.cc`: adds the ROS2 node, `controller_manager` node options, plugin loading, and the plugin hooks inside `PhysicsLoop`.

### Maintenance / git strategy (important)
ROS-related changes are layered at the **top (newest)** of the commit history. Syncing the `simulate` app to a new MuJoCo release is done by **rebasing at the bottom (oldest)** of history, which rewrites history. Because of this, keep ROS changes isolated to `main.cc` and the plugin header, and keep them as self-contained commits so the rebase stays clean. The package does **not** require a strict version match with the main MuJoCo library.

## Architecture

`main.cc` orchestrates two threads (upstream `simulate` design):

- **Main thread:** `sim->RenderLoop()` — GLFW/OpenGL UI, blocking.
- **Physics thread:** `PhysicsThread` → `PhysicsLoop` — loads the model, manages CPU/sim time sync, calls `mj_step`.

The ROS2 integration adds:

1. A node `mujoco_sim_ros2_node` and `controller_manager::get_cm_node_options()` so `--ros-args --param-file ...` passed on the command line reach the controller manager.
2. Parameters: `model_package` (ament package name), `model_file` (path relative to that package's share dir), and `physics_plugins` (list of pluginlib class names).
3. Plugins loaded via `pluginlib::ClassLoader<mujoco_sim_ros2::MujocoPhysicsPlugin>`, then `Configure(node, cm_node_options, m, d)` is called on each.

### Plugin lifecycle (`MujocoPhysicsPlugin`)
- `Configure(node, cm_node_option, model, data)` — once after load (pure virtual).
- `Reset(model, data)` — pure virtual.
- Per physics step, when any plugin is registered, `PhysicsLoop` replaces the plain `mj_step` with: `PreUpdate()` → `mj_step1()` → `Update()` → `mj_step2()`. `PreUpdate`/`Update`/`PostUpdate` are optional (default no-op) virtuals. **Note:** `PostUpdate` is declared but currently not invoked by `PhysicsLoop`.

## Build

This is one package inside a larger colcon workspace. It depends on sibling repos cloned next to it:
```bash
cd <colcon-ws>/src
git clone -b 3.9.0 https://github.com/google-deepmind/mujoco.git
git clone https://github.com/rxdu/mujoco_sim_ros2.git
git clone https://github.com/rxdu/mujoco_ros2_control.git
git clone https://github.com/rxdu/mujoco_demo_robot.git   # optional working example
```

Build from the workspace root (per repo / global ROS2 conventions):
```bash
colcon build --symlink-install --packages-up-to mujoco_sim_ros2 \
  --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja
```
CI (`.github/workflows/default.yaml`, `ros:jazzy` container) builds the whole workspace with `colcon build --symlink-install --cmake-args " -DCMAKE_BUILD_TYPE=Release"` after `rosdep install`. System deps include `libglfw3-dev libxinerama-dev libxcursor-dev libxi-dev` and `ros-jazzy-ros2-control ros-jazzy-ros2-controllers`.

`lodepng` is fetched at configure time via CMake `FetchContent`.

## Lint / test

`BUILD_TESTING` wires up `ament_lint_auto`, but copyright and cpplint checks are **deliberately disabled** in `CMakeLists.txt` (no license header on every source file yet). Run lint via colcon:
```bash
colcon test --packages-select mujoco_sim_ros2
colcon test-result --verbose
```
There are currently no unit tests — verification is done by building and running the demo (below).

## Run

```bash
ros2 launch mujoco_demo_robot cart_effort.launch.py
# inspect the ros2_control setup:
ros2 control list_hardware_interfaces
ros2 control list_controllers
# drive the demo:
ros2 run mujoco_ros2_control_demos example_effort
```
The actual robot model (`test_cart.xml`) lives in `mujoco_ros2_control`; `mujoco_demo_robot` provides the launch files that set `model_package`, `model_file`, and `physics_plugins`.
