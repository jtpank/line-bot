# Paint Robot Dora Simulation

This is a first-pass simulation scaffold for a four-wheel paint robot:

- RTK GPS and IMU are simulated as separate C++ Dora nodes.
- The two rear drive motors are represented by `drive_command`; the front support is modeled as a passive ball/caster, so the robot uses differential-drive/tank steering and can rotate in place.
- The paint valve is represented by a `paint_command` boolean and line id, with the paint nozzle mounted at a fixed offset from the robot center.
- The planner chooses map line segments to paint, tracks completed segments, stays inside keep-in boundaries, avoids completed paint, marks infeasible lines with reasons, and routes home when the mission is accounted for.

The map file in `maps/demo_track.map` is a text-based parking-lot striping demo with ten spaces and two painted side lines per space. It accepts surveyed-style GPS inputs, configurable distance units, paint-line length, and keep-in boundaries such as curbs.

## Map Format

The current map parser supports both the older local-meter format and the newer GPS format. The simulation always normalizes to local meters internally.

```text
type=mission
units=feet
gps_origin=lat,lon,alt
home_gps=lat,lon,alt,yaw_rad
segment_gps=id,start_lat,start_lon,start_alt,bearing_deg_clockwise_from_north,length
boundary_gps=id,kind,lat,lon,alt|lat,lon,alt|lat,lon,alt
paint_arm_offset=forward,left,z
sim_time_scale=1.0
```

`units` can be `meters` or `feet`; it applies to unitless distance fields such as `track_width`, `paint_width`, `avoid_clearance`, and `segment_gps` length. Explicit SI fields such as `track_width_m` and `max_speed_mps` still work.

`paint_arm_offset` is measured in the configured map units. It describes the static nozzle position relative to robot center as `forward,left,z`. Use `paint_arm_offset_m` to specify the same values explicitly in meters. The planner paints with the nozzle path while routing the robot center so the fixed arm offset does not push the body through boundaries.

`sim_time_scale` speeds up or slows down simulated vehicle time without changing the Dora tick rate. The included demo map uses `1.0` so `dataflow.yml` has a sane closed-loop control rate. `dataflow_fast.yml` overrides it with `SIM_TIME_SCALE=4.0` for accelerated visualization.

Boundary `kind` can be `keep_in` or `keep_out`. Keep-in polygons define valid operating areas; keep-out polygons define internal obstacles or exclusion zones inside the map. Boundaries can be interior obstacles, not only exterior map edges. `line_boundary_buffer` reserves distance between each line end and any boundary edge. The planner only approaches each line from its start point and accounts for every line as either completed or infeasible. Infeasible reasons are currently `boundary_proximity`, `fresh_paint_crossing`, and `paint_arm_unreachable`. `vehicle_sim` refuses position updates that would leave valid space.

## Dependencies

This repo builds C++20 Dora nodes and a small Rust static library for Dora's C node API. The viewer is a Python Dora node that renders into `dora-rerun`.

Required tools:

- C++20 compiler: `g++` or `clang++`
- CMake `3.20` or newer
- Rust/Cargo
- Dora CLI, pinned to the same `0.5.x` family as `dora-node-api-c`
- `uv` for the Python viewer environment
- Python `3.12` for the viewer dataflows

On Linux or Jetson Ubuntu:

```bash
sudo apt update
sudo apt install -y build-essential pkg-config curl git ca-certificates libssl-dev python3 python3-venv python3-pip

# Install CMake from apt only if it is new enough for this repo.
sudo apt install -y cmake
cmake --version
```

If `cmake --version` reports older than `3.20`, install a newer CMake before configuring the project. JetPack 5 / Ubuntu 20.04 images commonly need this; JetPack 6 / Ubuntu 22.04 is more likely to have a new enough CMake available. One straightforward fallback is:

```bash
python3 -m pip install --user --upgrade cmake
export PATH="$HOME/.local/bin:$PATH"
cmake --version
```

Install Rust, Dora, and `uv`:

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
export PATH="$HOME/.cargo/bin:$HOME/.local/bin:$PATH"
cargo install dora-cli --version 0.5.0
cargo install uv
dora --version
uv --version
```

On macOS with Homebrew:

```bash
brew install cmake rust python@3.12 uv
export PATH="$HOME/.cargo/bin:/opt/homebrew/bin:$PATH"
cargo install dora-cli --version 0.5.0
dora --version
uv --version
```

The viewer dependencies are downloaded by `dora build` into `.venv` using the commands in the dataflow files:

```bash
uv venv --seed -p 3.12 .venv
uv pip install --python .venv/bin/python dora-rs pyarrow numpy pillow dora-rerun rerun-sdk==0.24.1
```

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

On Linux or Jetson, use `nproc` to pick a parallel build count:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel "$(nproc)"
```

Keep the build directory named `build` unless you also update the dataflow paths; the current dataflows launch binaries from `build/bin/...`.

The build compiles the Rust Dora C API static library first, then links the C++ nodes against it.

## Run

Build the Dora dataflow, which also creates the Python viewer environment:

```bash
dora build dataflow.yml
dora run dataflow.yml
```

`dora build` uses the build command on `map_server`; `dora run` starts the C++ node graph and viewer. On Linux/Jetson, make sure the shell running Dora has Cargo and local Python tools on `PATH`:

```bash
export PATH="$HOME/.cargo/bin:$HOME/.local/bin:$PATH"
```

For a faster visual demo, use the higher-rate dataflow:

```bash
dora build dataflow_fast.yml
dora run dataflow_fast.yml
```

For the physical robot target, assume Linux/aarch64 on the Jetson. The same C++ build steps apply there. The Rerun viewer is useful during development, but on a headless robot you will likely run the control/sensing nodes without the viewer node or stream visualization to a separate development machine.

## Dataflow

- `map_server`: loads GPS/local line segments, boundaries, and simulation limits from `MAP_PATH`, publishes `mission`.
- `vehicle_sim`: differential-drive vehicle dynamics for two rear motors plus a passive front ball/caster.
- `gps_sim`: publishes RTK-like pose from the simulated truth state.
- `imu_sim`: publishes yaw, yaw rate, and forward acceleration from simulated truth.
- `state_estimator`: emits a simple fused state estimate, preferring RTK pose when available.
- `paint_tracker`: marks segments complete when paint coverage spans the segment, using the nozzle position instead of robot center.
- `planner`: selects the next unpainted segment from the start side, routes the robot center so the fixed paint arm can reach the stripe, stays inside boundaries, avoids completed segments, marks infeasible lines with reason codes, and returns home.
- `controller`: converts plans into left/right rear motor speeds and valve commands.
- `monitor`: prints compact mission progress.
- `rerun_image_bridge`: renders the C++ simulation messages into a documented Dora image/text stream.
- `dora-rerun`: opens the official Dora Rerun visualization node.

## Viewer

The dataflow uses Dora's packaged `dora-rerun` visualization node. The bridge node publishes an `image` output as a `UInt8Array` with `primitive=image`, `width`, `height`, and `encoding=bgr8` metadata, plus a `text` output with mission status. The view shows the robot center, the fixed paint arm/nozzle position, completed paint, keep-in and keep-out boundaries, and infeasible lines colored by reason.

Run the same command:

```bash
dora build dataflow.yml
dora run dataflow.yml
```

Or use `dataflow_fast.yml` for the faster tick setup.

The Rerun viewer should open when the dataflow starts. If macOS blocks window launch from the sandbox, run the same `dora run dataflow.yml` command from a normal terminal in this directory.

## Refinement Needed

Before this becomes hardware-faithful, the open items are:

- final production map format and coordinate-frame details;
- exact boundary semantics for curbs, wheel footprints, and temporary exclusion zones;
- robot rear track width, speed limits, and acceleration limits;
- RTK/IMU noise and update rates;
- what exactly counts as "driving over" completed paint: current check uses robot-center route samples as a first proxy, but wheel footprint or whole robot footprint would be more realistic;
- paint completion tolerance and line-width model.
