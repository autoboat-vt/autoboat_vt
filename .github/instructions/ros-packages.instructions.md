---
description: "Use when writing or editing ROS 2 packages under ros_packages/. Covers ament_python vs ament_cmake layouts, autoboat_msgs (.msg files + rosidl_generate_interfaces), launch file patterns (respawn, ros_gz_bridge topic syntax, micro_ros_agent), default params JSON format, simulation Gazebo plugins (custom_lift_drag CMakeLists for gz-sim8), simulation_models ament_python SDF shipping, and topic name conventions."
applyTo: "ros_packages/**"
---

# ROS 2 Packages Conventions

All ROS 2 packages live under `ros_packages/`. Two build types:

| Type | Used for | Layout |
|------|----------|--------|
| `ament_python` | Pure-Python packages (`autopilot`, `drivers`, `object_detection`, `autoboat_launch`, `simulation_models`) | `package.xml`, `setup.py`, `setup.cfg`, `resource/<pkg>`, same-named Python dir |
| `ament_cmake` | C++ packages (`autopilot_cpp`, `drivers_cpp`, `autoboat_msgs`, simulation plugins) | `package.xml`, `CMakeLists.txt`, `src/` or `include/`+`src/` |

All nodes publish/subscribe on `ROS_DOMAIN_ID=42`, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` (set in the devcontainer env). The `old_sailboat_simulation/` package is **deprecated** — prefer `ros_packages/simulation/` for new simulation work.

## `autoboat_msgs` — defining new messages

File: `ros_packages/autoboat_msgs/CMakeLists.txt` + `package.xml`.

### `CMakeLists.txt` (canonical invocation)

```cmake
cmake_minimum_required(VERSION 3.8)
project(autoboat_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

set(msg_files
  "msg/VESCTelemetryData.msg"
  "msg/VESCControlData.msg"
  "msg/RCData.msg"
  "msg/WaypointList.msg"
  "msg/ObjectDetectionResult.msg"
  "msg/ObjectDetectionResultsList.msg"
  "msg/TriangulationResult.msg"
  "msg/TriangulationResultsList.msg"
  "msg/JetsonStats.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

### `package.xml`

```xml
<package format="3">
  <name>autoboat_msgs</name>
  <version>1.0.0</version>
  <description>Contains all of the custom message types that the autopilot or sensors may use</description>
  <maintainer email="autoboat@vt.edu">autoboatvt</maintainer>
  <license>http://www.apache.org/licenses/LICENSE-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### Pattern notes

- `rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files} DEPENDENCIES sensor_msgs)` is the canonical invocation.
- `WaypointList.msg` uses `sensor_msgs/NavSatFix[]` — hence the `DEPENDENCIES sensor_msgs` and the `sensor_msgs` `find_package`.
- `member_of_group rosidl_interface_packages` is required for any package exporting messages.
- The two `set(ament_cmake_*_FOUND TRUE)` lines disable `copyright` and `cpplint` linters (they're not in `ament_lint_common`).
- **Adding a new `.msg` file requires a colcon rebuild AND downstream C++/Python updates** (include the new header in C++ or import in Python).

## `.msg` file conventions

### Nested types reference their own package as `autoboat_msgs/Foo[]`

```
# ObjectDetectionResultsList.msg
autoboat_msgs/ObjectDetectionResult[] detection_results
int64 ntp_timestamp
string model_name
int8 yolo_version
float32 threshold
```

### External types referenced as `sensor_msgs/NavSatFix[]`

```
# WaypointList.msg
sensor_msgs/NavSatFix[]     waypoints
```

Must be added to `DEPENDENCIES` in `rosidl_generate_interfaces`.

### Field type mix

`float32`, `int32`, `int8`, `bool`, `string`, `int64` — choose the smallest type that fits. `int8` for small enums/versions, `int64` for timestamps (nanoseconds since epoch).

### All current `.msg` files

| File | Purpose |
|------|---------|
| `JetsonStats.msg` | Jetson power/temp/CPU/GPU/RAM stats |
| `ObjectDetectionResult.msg` | Single detection (confidence, x/y/width/height, object_id, class_id, angle_to_object) |
| `ObjectDetectionResultsList.msg` | Frame of detections + ntp_timestamp + model_name + yolo_version + threshold |
| `RCData.msg` | RC joystick axes + buttons/toggles |
| `TriangulationResult.msg` | Triangulated object lat/lon + label |
| `TriangulationResultsList.msg` | List of triangulation results + iou_threshold |
| `VESCControlData.msg` | VESC control command (control_type string + value) |
| `VESCTelemetryData.msg` | VESC telemetry (rpm, duty, voltages, currents, temps, amp_hours) |
| `WaypointList.msg` | `sensor_msgs/NavSatFix[]` waypoints |

## Default parameter JSON format

Files: `ros_packages/autopilot/autopilot/config/{motorboat,sailboat}_default_parameters.json`. Every parameter is a dict with `default` (scalar or list — supports arrays for lookup tables) + `description`:

```json
{
    "autopilot_refresh_rate": {
        "default": 50.0,
        "description": "Refresh rate for the autopilot in seconds."
    },
    "sail_lookup_table_wind_angles": {
        "default": [0, 45, 90, 135, 180, 225, 270, 315, 360],
        "description": "Wind angles for the lookup table in degrees."
    },
    "sail_lookup_table_sail_positions": {
        "default": [70.0, 50.0, 30.0, 10.0, 0.0, 10.0, 30.0, 50.0, 70.0],
        "description": "Sail positions corresponding to the wind angles in the lookup table."
    }
}
```

The ground station `AutopilotConfigEditor` reads this format and renders one `AutopilotParamWidget` per entry; lists are edited via `edit_grouped_data` → `TextEditWindow`. The C++ node loads it via `json::parse(std::ifstream(...))` and iterates `config.begin()...config.end()` taking `it.key()` → `it.value()["default"]`.

## Launch file patterns

Launch files live in `ros_packages/autoboat_launch/autoboat_launch/`. Variants: `motorboat.launch.py`, `motorboat_simulation.launch.py`, `motorboat_simulation_cpp.launch.py`, `sailbot_jetson.launch.py`, `sailboat_simulation.launch.py`, `sailboat_simulation_cpp.launch.py`, `motorboat_simulation_differential_thrust.launch.py`, `camera_testing.launch.py`. The `_cpp` variants use `autopilot_cpp` instead of Python `autopilot`.

### Real hardware launch (`motorboat.launch.py` representative)

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="autopilot",
                executable="motorboat_autopilot",
                name="motorboat_autopilot",
                respawn=True,
                respawn_delay=2.0,
                # output="log"
            ),
            Node(package="autopilot", executable="telemetry", name="telemetry",
                 respawn=True, respawn_delay=2.0),
            Node(package="drivers", executable="gps", name="gps",
                 respawn=True, respawn_delay=2.0),
            Node(package="drivers", executable="rc", name="rc",
                 respawn=True, respawn_delay=2.0),
            Node(package="drivers", executable="vesc", name="vesc",
                 respawn=True, respawn_delay=0.01),
            Node(
                package="micro_ros_agent",
                executable="micro_ros_agent",
                name="micro_ros_agent",
                output="screen",
                arguments=["serial", "--dev", "/dev/pico", "-b", "115200"],
            ),
        ]
    )
```

**Pattern notes:**
- Each `Node` uses `respawn=True, respawn_delay=2.0` (VESC uses `0.01` — it crashes on startup more often).
- `output` left commented to default (`log`); `micro_ros_agent` set to `"screen"` for debugging.
- `micro_ros_agent` connects to the Pico RP2040 over serial at `/dev/pico` baud `115200`.
- Sailboat variant swaps `motorboat_autopilot`→`sailboat_autopilot`, drops `vesc` (sailboats have no VESC ESC), adds `wind_sensor` driver.

### Gazebo simulation launch (`motorboat_simulation.launch.py` representative)

```python
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')
    gz_launch_path = PathJoinSubstitution(
        [ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py'])

    simulation_models_pkg_path = get_package_share_directory('simulation_models')
    model_path = PathJoinSubstitution(
        [simulation_models_pkg_path, 'models', 'motorboat_model.sdf'])

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': ["-s --headless-rendering -r ", model_path],
                'on_exit_shutdown': 'True'
            }.items(),
        ),
        Node(package='simulation_transform', executable='simulation_transform',
             name='simulation_transform', respawn=True, respawn_delay=2.0),
        Node(package='autopilot', executable='telemetry', name='telemetry',
             respawn=True, respawn_delay=2.0, output="log"),
        Node(package='autopilot', executable='motorboat_autopilot',
             name='motorboat_autopilot', respawn=True, respawn_delay=2.0,
             output="screen"),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/motorboat_simulation/desired_propeller_rpm@std_msgs/msg/Float64]gz.msgs.Double',
                '/motorboat_simulation/desired_rudder_angle@std_msgs/msg/Float64]gz.msgs.Double',
                '/motorboat_simulation/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/motorboat_simulation/position@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
                '/world/motorboat_model/model/my_ship/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'
            ],
            remappings=[('/motorboat_simulation/position', '/position')],
            output='screen'
        ),
    ])
```

**Pattern notes:**
- Gazebo launched via `IncludeLaunchDescription` of `ros_gz_sim`'s `gz_sim.launch.py`, with `gz_args` = `-s --headless-rendering -r <model.sdf>`. `-s` = server only, `-r` = run on load.
- SDF model path resolved through `get_package_share_directory('simulation_models')` + `models/<name>.sdf`.
- `ros_gz_bridge` `parameter_bridge` topics use the `topic@ros_type[direction]gz_type` syntax:
  - `]` after the ROS type = **ROS→GZ** (publisher side, ROS publishes to GZ).
  - `[` after the ROS type = **GZ→ROS** (subscriber side, GZ publishes to ROS).
- `remappings=[('/motorboat_simulation/position', '/position')]` is how sim topics get renamed to the canonical hardware topic names.

## Simulation Gazebo plugins

`ros_packages/simulation/` contains Gazebo Harmonic plugins. Each is an `ament_cmake` package building a `SHARED` library.

### `custom_lift_drag/CMakeLists.txt` (representative)

```cmake
cmake_minimum_required(VERSION 3.10)
project(custom_lift_drag)

# Gazebo Harmonic uses gz-sim8 and gz-plugin2
find_package(gz-sim8 REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(ament_cmake REQUIRED)

set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

add_library(CustomLiftDrag SHARED src/CustomLiftDrag.cc)
set_property(TARGET CustomLiftDrag PROPERTY CXX_STANDARD 17)

target_link_libraries(CustomLiftDrag
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-common5::gz-common5
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
)

find_package(rclcpp REQUIRED)
target_link_libraries(CustomLiftDrag rclcpp::rclcpp)

install(TARGETS CustomLiftDrag LIBRARY DESTINATION lib)

ament_package()
```

### Plugin conventions

- Targets **Gazebo Harmonic** specifically: `gz-sim8`, `gz-common5`, `gz-plugin2`. The major-version variables (`GZ_SIM_VER`, `GZ_PLUGIN_VER`) make the target link names version-agnostic.
- Plugin = a `SHARED` library, C++17, installed to `lib/`.
- `package.xml` is minimal — just `ament_cmake` buildtool, no `<depend>` entries for the gz packages (they're found via CMake).
- **`GZ_SIM_SYSTEM_PLUGIN_PATH`** (set in `.devcontainer/postCreateCommand.sh`) MUST include the build dir for each plugin (`build/custom_lift_drag/`, `build/foil_dynamics/`, `build/sail_limits/`, `build/wind_arrow/`) so Gazebo can locate the `.so` at runtime — **new sim plugins MUST be added there** or Gazebo won't load them.

## `simulation_models` — SDF/URDF models

`ros_packages/simulation/simulation_models/` is an **`ament_python`** package (note `<build_type>ament_python</build_type>` in `package.xml`, with `setup.py`/`setup.cfg`/`resource/`). The SDF models live in `models/` and are installed to the package's share directory, then resolved at launch time via `get_package_share_directory('simulation_models')` + `models/<name>.sdf`.

```
simulation_models/
    models/
        motorboat_model.sdf
        motorboat_model_differential_thrust.sdf
        sailboat_model.sdf
    package.xml
    resource/
    setup.cfg
    setup.py
```

> ⚠️ `simulation_models/package.xml` build deps are unusual — they declare `ament_copyright`, `ament_flake8`, `ament_pep257`, `python3-pytest` as `<buildtool_depend>` (rather than `<test_depend>`), which is non-standard but functional.

## Topic name conventions

Canonical topics used across hardware + sim (sim topics remapped to these):

| Topic | Type | Notes |
|-------|------|-------|
| `/position` | `sensor_msgs/NavSatFix` | GPS position |
| `/heading` | `std_msgs/Float32` | Compass heading (deg) |
| `/desired_rudder_angle` | `std_msgs/Float32` | Autopilot → firmware |
| `/current_rudder_angle` | `std_msgs/Float32` | Firmware → autopilot (encoder) |
| `/desired_sail_angle` | `std_msgs/Float32` | Sailboat only |
| `/rc_data` | `autoboat_msgs/RCData` | RC receiver → autopilot |
| `/waypoints` | `autoboat_msgs/WaypointList` | Ground station → autopilot |
| `/autopilot_parameters` | `std_msgs/String` (JSON) | Ground station → autopilot |
| `/autopilot_param_config_path` | `std_msgs/String` (transient_local) | Autopilot → ground station |
| `/motorboat_simulation/*` | various | Sim-only, bridged to canonical names via `remappings` |

## Things to avoid

- Editing files under `build/`, `install/`, `log/` — colcon build artifacts.
- Adding a new `.msg` without rebuilding colcon AND updating downstream C++/Python.
- Using `member_of_group rosidl_interface_packages` for non-message packages.
- Adding a Gazebo plugin without also adding its build dir to `GZ_SIM_SYSTEM_PLUGIN_PATH` in `.devcontainer/postCreateCommand.sh`.
- Using `output="screen"` for high-rate nodes (fills the terminal); prefer `"log"` or commented-out (default).
- Putting sim work in `old_sailboat_simulation/` — it's deprecated; use `ros_packages/simulation/`.
- Forgetting `remappings` when bridging sim topics to canonical names — autopilot nodes expect `/position` not `/motorboat_simulation/position`.
- Declaring `nlohmann_json`/`OpenSSL`/`cpr`/`serial_driver`/`udev` in `package.xml` — they resolve via devcontainer system installs (see `cpp.instructions.md`).
