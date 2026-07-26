---
description: "Use when writing or editing C++ in autopilot_cpp, drivers_cpp, or firmware. Covers CMakeLists (shared lib + PCH + REUSE_FROM, ccache/mold auto-detect), C++23 vs C++17 firmware split, node skeletons (vector of subscriptions, set__data setters, transient_local QoS), library API (DiscretePID, Position, SailboatAutopilot, MotorboatAutopilot, autopilot_utils enums, geographic_function_library, telemetry_payloads packed structs), drivers_cpp multi-sub-node pattern, and package.xml gotchas."
applyTo: "**/*.cpp, **/*.hpp, **/*.h, **/*.cc, **/CMakeLists.txt"
---

# C++ Conventions

Two C++ surfaces:

| Path | Standard | Stack |
|------|----------|-------|
| `ros_packages/autopilot_cpp/` | **C++23** | ROS 2 Humble, ament_cmake |
| `ros_packages/drivers_cpp/` | **C++23** | ROS 2 Humble, ament_cmake, serial_driver |
| `firmware/src/` | **C++17** | RP2040 Pico SDK + micro-ROS (see `firmware.instructions.md`) |

Compiler flags: `-Wall -Wextra -Wpedantic -ffunction-sections -fdata-sections` + `-Wl,--gc-sections`. Use `ccache` and `mold` when available (auto-detected by CMake).

## Workflow

C++ changes **always** require a rebuild (unlike Python with `--symlink-install`):

```bash
build          # full (Python + C++)
build_python   # skip *_cpp packages — faster during Python-only work
```

These are aliases defined in `.devcontainer/postCreateCommand.sh` that expand to `cd /home/ws && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`.

## `autopilot_cpp/CMakeLists.txt` — shared lib + PCH + REUSE_FROM

File: `ros_packages/autopilot_cpp/CMakeLists.txt`. Key structure:

- **C++23**, `CMAKE_EXPORT_COMPILE_COMMANDS ON`.
- Auto-detects **ccache** (compiler launcher) and **mold** (linker).
- `-Wall -Wextra -Wpedantic -ffunction-sections -fdata-sections` + `-Wl,--gc-sections`.
- **Shared static lib** `autopilot_lib` compiles all `autopilot_library/*.cpp` once, with `target_precompile_headers` for `<rclcpp/rclcpp.hpp>` and `<nlohmann/json.hpp>`. Three executables **reuse** those precompiled headers via `REUSE_FROM`.
- **FetchContent** pulls `cpr` v1.14.2 (uses system curl).
- `OpenCV_STATIC ON`.
- `install(DIRECTORY config ...)` ships the JSON default-parameter files.

### Toolchain auto-detection (verbatim)

```cmake
# Use ccache if available (huge incremental build speedup)
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
endif()

# Use mold linker if available (faster linking)
find_program(MOLD_PROGRAM mold)
if(MOLD_PROGRAM)
  add_link_options(-fuse-ld=mold)
endif()
```

### Shared library + PCH (verbatim)

```cmake
add_library(autopilot_lib STATIC
  autopilot_library/geographic_function_library.cpp
  autopilot_library/autopilot_utils.cpp
  autopilot_library/discrete_pid.cpp
  autopilot_library/position.cpp
  autopilot_library/motorboat_autopilot.cpp
  autopilot_library/sailboat_autopilot.cpp
  autopilot_library/telemetry_payloads.cpp
)
target_include_directories(autopilot_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${CMAKE_CURRENT_SOURCE_DIR}
)
ament_target_dependencies(autopilot_lib rclcpp std_msgs geometry_msgs sensor_msgs autoboat_msgs)
target_link_libraries(autopilot_lib nlohmann_json::nlohmann_json)

target_precompile_headers(autopilot_lib PRIVATE
  <rclcpp/rclcpp.hpp>
  <nlohmann/json.hpp>
)
```

### Executable that reuses PCH + links the static lib (verbatim)

```cmake
add_executable(motorboat_autopilot src/motorboat_autopilot_node.cpp)
target_include_directories(motorboat_autopilot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${CMAKE_CURRENT_SOURCE_DIR}
)
ament_target_dependencies(motorboat_autopilot rclcpp std_msgs geometry_msgs sensor_msgs autoboat_msgs)
target_link_libraries(motorboat_autopilot autopilot_lib nlohmann_json::nlohmann_json)
target_precompile_headers(motorboat_autopilot REUSE_FROM autopilot_lib)
install(TARGETS motorboat_autopilot DESTINATION lib/${PROJECT_NAME})
```

The `telemetry` executable additionally links `cpr::cpr OpenSSL::SSL OpenSSL::Crypto ${OpenCV_LIBRARIES}` and adds `PRIVATE ${OpenCV_INCLUDE_DIRS}`.

### `package.xml` gotcha

`ros_packages/autopilot_cpp/package.xml` uses only `<depend>` entries:

```xml
<depend>autoboat_msgs</depend>
<depend>rclcpp</depend>
<depend>std_msgs</depend>
<depend>geometry_msgs</depend>
<depend>sensor_msgs</depend>
<depend>opencv</depend>
```

> ⚠️ **`nlohmann_json`, `OpenSSL`, and `cpr` are NOT declared** in `package.xml` despite being `find_package`d — they resolve via the devcontainer's system installs. Adding a new external C++ dep requires both a `find_package` in CMake **and** a `<depend>` (or system-package dependency) here. There is no `build_export_depend`/`exec_depend` split — just `<depend>`.

## Node skeleton (`MotorboatAutopilotNode` representative)

File: `ros_packages/autopilot_cpp/src/motorboat_autopilot_node.hpp`. Class declaration pattern (`public rclcpp::Node`):

```cpp
class MotorboatAutopilotNode : public rclcpp::Node {
public:
    MotorboatAutopilotNode();

private:
    MotorboatAutopilot motorboat_autopilot;
    std::map<std::string, json> autopilot_parameters;
    MotorboatControlModes autopilot_mode = MotorboatControlModes::WAYPOINT_MISSION;
    PropellerMotorControlMode propeller_motor_control_mode = PropellerMotorControlMode::RPM;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr current_waypoint_index_publisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr desired_rudder_angle_publisher;
    rclcpp::Publisher<autoboat_msgs::msg::VESCControlData>::SharedPtr propeller_motor_control_struct_publisher;
    // ... more publishers ...

    rclcpp::TimerBase::SharedPtr autopilot_refresh_timer;
    rclcpp::Time last_rc_data_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
    // ... more state ...

    std::vector<rclcpp::SubscriptionBase::SharedPtr> sub;

    void position_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void rc_data_callback(const autoboat_msgs::msg::RCData::SharedPtr msg);
    void autopilot_parameters_callback(const std_msgs::msg::String::SharedPtr new_parameters);
    void waypoints_list_callback(const autoboat_msgs::msg::WaypointList::SharedPtr waypoint_list);
    void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void update_ros_topics();
};
```

### Constructor — load JSON defaults, publish config path, build subs/pubs

```cpp
MotorboatAutopilotNode::MotorboatAutopilotNode() : Node("motorboat_autopilot_cpp") {
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("autopilot_cpp");
    std::string json_file_path = package_share_directory + "/config/motorboat_default_parameters.json";

    std::ifstream file(json_file_path);
    json config = json::parse(file);

    for (auto it = config.begin(); it != config.end(); ++it) {
        autopilot_parameters[it.key()] = it.value()["default"];
    }

    auto transient_qos = rclcpp::QoS(1).reliable().transient_local();
    config_path_publisher = this->create_publisher<std_msgs::msg::String>(
        "/autopilot_param_config_path", transient_qos);
    config_path_publisher->publish(std_msgs::msg::String().set__data(json_file_path));

    motorboat_autopilot = MotorboatAutopilot(&autopilot_parameters);

    rclcpp::SensorDataQoS sensor_qos = rclcpp::SensorDataQoS();

    autopilot_refresh_timer = create_wall_timer(
        std::chrono::duration<float>(1.0 / autopilot_parameters["autopilot_refresh_rate"].get<float>()),
        std::bind(&MotorboatAutopilotNode::update_ros_topics, this)
    );

    // Subscriptions pushed into a vector (keeps them alive)
    sub.push_back(create_subscription<std_msgs::msg::String>(
        "/autopilot_parameters", 10,
        std::bind(&MotorboatAutopilotNode::autopilot_parameters_callback, this, _1)));
    sub.push_back(create_subscription<sensor_msgs::msg::NavSatFix>(
        "/position", 10,
        std::bind(&MotorboatAutopilotNode::position_callback, this, _1)));
    sub.push_back(create_subscription<autoboat_msgs::msg::RCData>(
        "/rc_data", sensor_qos,
        std::bind(&MotorboatAutopilotNode::rc_data_callback, this, _1)));
    // ...
}
```

### `main()`

```cpp
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorboatAutopilotNode>());
    rclcpp::shutdown();
    return 0;
}
```

### Key conventions visible

- **Subscriptions stored in a `std::vector<rclcpp::SubscriptionBase::SharedPtr> sub;`** (keeps shared_ptrs alive; alternative to member-per-sub).
- `set__data(...)` setter style for messages (ROS 2 generated setters).
- Structured bindings (`auto [desired_rpm, desired_rudder] = ...`) and `std::optional` returns from the library.
- Timer callback `update_ros_topics()` is the central tick: reads state, dispatches on `autopilot_mode` enum, publishes.
- Dynamic timer refresh rate (cancels + recreates timer when `autopilot_refresh_rate` param changes).
- Library classes take `std::map<std::string, json> *autopilot_parameters` (non-owning) — the **node owns the params map** and the library reads from it live.

## Library API surface (`autopilot_library/`)

All library classes take `std::map<std::string, json> *autopilot_parameters` (non-owning). Return types favour `std::pair`/`std::tuple`/`std::optional` over out-params. `discrete_pid`, `position`, `geographic_function_library`, `autopilot_utils` have **no ROS deps** — pure logic, unit-testable. The `*_autopilot` classes pull in ROS transitively only via `autopilot_utils.hpp` → `ament_index_cpp`.

### `discrete_pid.hpp`

```cpp
class DiscretePID {
public:
    DiscretePID();
    DiscretePID(float sample_period_, float Kp_, float Ki_, float Kd_, float n_);
    void set_gains(float sample_period_ = -1.0, float Kp_ = -1.0, float Ki_ = -1.0, float Kd_ = -1.0, float n_ = -1.0);
    void reset();
    float step(float error);
    float operator()(float error);
};
```

Low-pass-filtered derivative term; `operator()` aliases `step()`. Negative arg to `set_gains` = "keep existing value".

### `position.hpp` — arg-order-flip gotcha

```cpp
class Position {
public:
    double longitude;
    double latitude;

    Position(double longitude, double latitude);                                    // global init
    Position(float local_x, float local_y, double reference_longitude, double reference_latitude);  // local NED init

    void set_longitude_latitude(double longitude, double latitude);
    std::array<double, 2> get_longitude_latitude() const;     // [longitude, latitude]
    std::array<double, 2> get_latitude_longitude() const;     // [latitude, longitude]
    void set_local_coordinates(float local_x, float local_y, double reference_longitude, double reference_latitude);
    std::array<float, 2> get_local_coordinates(const std::array<double, 2>& reference_longitude_latitude) const;
};
```

> ⚠️ **Argument order flips:** global constructor is `(longitude, latitude)`; `get_longitude_latitude()` returns `[longitude, latitude]` but `get_latitude_longitude()` returns `[latitude, longitude]`. Read the signature carefully.

### `sailboat_autopilot.hpp`

```cpp
class SailboatAutopilot {
private:
    std::map<std::string, json> *autopilot_parameters;     // NON-OWNING pointer
    std::vector<Position> waypoints;
    int current_waypoint_index;
    float desired_tacking_angle = 0.0f;
    SailboatAutopilotStates current_state = SailboatAutopilotStates::DOWNWIND_SAILING;
    Position last_tacking_position = Position(0.0, 0.0);
    std::chrono::steady_clock::time_point last_time_out_of_no_sail_zone;
    // ... private helpers ...

public:
    DiscretePID heading_pid_controller;     // PUBLIC member
    float heading_to_hold;

    SailboatAutopilot();
    SailboatAutopilot(std::map<std::string, json> *autopilot_parameters_);

    void reset();
    void update_waypoints_list(const std::vector<Position>& waypoints_list);
    float get_current_waypoint_index();
    std::vector<Position> get_current_waypoints_list();
    SailboatAutopilotStates get_current_waypoint_mission_state();

    std::pair<float, float> run_emergency_stop_step(float heading_object_was_detected_at, float current_heading, float apparent_wind_angle);
    float get_optimal_sail_angle(float apparent_wind_angle);
    float get_optimal_rudder_angle(float heading, float desired_heading);
    std::pair<float, float> run_rc_control(float rc_rudder_control, float rc_sail_control);
    std::tuple<std::optional<float>, std::optional<float>, std::optional<float>> run_waypoint_mission_step(
        Position current_position, std::array<float, 2> global_velocity_vector,
        float heading, std::array<float, 2> apparent_wind_vector);
};
```

### `motorboat_autopilot.hpp`

```cpp
class MotorboatAutopilot {
private:
    DiscretePID heading_pid_controller;
    std::map<std::string, json> *autopilot_parameters;     // NON-OWNING pointer
    std::vector<Position> waypoints;
    int current_waypoint_index = 0;

public:
    MotorboatAutopilot();
    MotorboatAutopilot(std::map<std::string, json> *autopilot_parameters_);

    int get_current_waypoint_index() const;
    std::vector<Position> get_current_waypoints_list() const;
    void reset();
    void update_waypoints_list(const std::vector<Position>& waypoints_list);

    float get_optimal_rudder_angle(float heading, float target_heading);
    std::tuple<std::string, float, float> run_rc_control(
        float joystick_left_y, float joystick_right_x,
        PropellerMotorControlMode propeller_motor_control_mode);
    float get_optimal_rpm(float rudder_angle);
    std::pair<float, std::optional<float>> run_waypoint_mission_step(Position current_position, float heading);
};
```

### `autopilot_utils.hpp` — shared enums (third copy of the cross-repo invariant)

```cpp
enum class SailboatControlModes      { DISABLED=0, FULL_RC=1, HOLD_BEST_SAIL=2, HOLD_HEADING=3, HOLD_HEADING_AND_BEST_SAIL=4, WAYPOINT_MISSION=5, EMERGENCY_STOP=6 };
enum class SailboatAutopilotStates   { NA=0, DOWNWIND_SAILING=1, PORT_TACK=2, STARBOARD_TACK=3, CW_TACKING=4, CCW_TACKING=5, STALL_WIGGLE_TO_PORT_TACK=6, STALL_WIGGLE_TO_STARBOARD_TACK=7 };
enum class SailboatManeuvers         { AUTOPILOT_DISABLED=0, STANDARD=1, TACK=2, JIBE=3 };
enum class MotorboatControlModes     { DISABLED=0, FULL_RC=1, HOLD_HEADING=2, WAYPOINT_MISSION=3, EMERGENCY_STOP=4 };
enum class PropellerMotorControlMode { RPM=0, DUTY_CYCLE=1, CURRENT=2 };

std::string to_string(MotorboatControlModes mode);
std::string to_string(SailboatControlModes mode);
std::string to_string(SailboatAutopilotStates state);

bool check_float_equivalence(float f1, float f2);
std::pair<float, float> cartesian_vector_to_polar(float x, float y);
float get_angle_between_vectors(const std::array<float, 2>& v1, const std::array<float, 2>& v2);
float get_distance_between_angles(float angle1, float angle2);
float get_bearing(const Position& current, const Position& dest);
```

> ⚠️ These are a **third copy** of the enum invariant — must stay aligned with `ros_packages/autopilot/autopilot/autopilot_library/utils/constants.py` AND `ground_station/src/utils/constants.py::StrictMatchEnums`. See `python.instructions.md` "Enum sync rule" for known drifts.

### `geographic_function_library.hpp` — pure functions, no state

```cpp
extern const std::string DISTANCE_FUNCTION_TO_USE;        // "vincenty" or "haversine"
constexpr double WGS84_A  = 6378137.0;
constexpr double WGS84_E2 = 6.69437999014e-3;
constexpr double WGS84_F  = 1 / 298.257223563;

float get_distance_haversine(double la1, double lo1, double la2, double lo2);
float get_distance_vincenty(double la1, double lo1, double la2, double lo2);
float get_distance(double la1, double lo1, double la2, double lo2);   // dispatches via DISTANCE_FUNCTION_TO_USE
float calculate_bearing(double la1, double lo1, double la2, double lo2);
std::array<double, 3> lla2ecef(double lat, double lon, double alt);
std::array<double, 3> ecef2lla(double x, double y, double z);
std::array<std::array<double, 3>, 3> nedRotation(double lat, double lon);
std::array<double, 3> matmul(const std::array<std::array<double,3>,3>&, const std::array<double,3>&);
// + ecef2ned, lla2ned, etc.
```

### `telemetry_payloads.hpp` — packed structs for wire transmission

```cpp
#pragma pack(push, 1)
struct BoatStatusPayloadBase {
    float latitude, longitude, distance_to_next_waypoint, speed;
    float velocity_x, velocity_y, desired_heading, heading;
    float desired_rudder_angle, current_rudder_angle, rudder_angle_error;
    uint8_t current_waypoint_index, boat_control_mode;
};
struct SailboatStatusPayload : public BoatStatusPayloadBase { /* wind + sail fields */ };
struct MotorboatStatusPayload : public BoatStatusPayloadBase { /* rpm, vesc telemetry */ };
#pragma pack(pop)

json get_sailboat_mapping_raw();
json get_motorboat_mapping_raw();
```

## `drivers_cpp/` — multi-sub-node package pattern

File: `ros_packages/drivers_cpp/CMakeLists.txt`. **One package, multiple sub-directories under `src/`, each with its own node executable + a shared protocol library.** C++23.

```cmake
include_directories(src)
include_directories(src/rc)
include_directories(src/motor_controller)

# motor_controller: VESC
add_library(vesc_protocol SHARED
  src/motor_controller/vesc_protocol/buffer.cpp
  src/motor_controller/vesc_protocol/crc.cpp
  src/motor_controller/vesc_protocol/vesc_protocol.cpp)
add_executable(vesc_node src/motor_controller/vesc_node.cpp)
target_link_libraries(vesc_node vesc_protocol ${UDEV_LIB})
ament_target_dependencies(vesc_node rclcpp std_msgs autoboat_msgs serial_driver)
install(TARGETS vesc_protocol ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS vesc_node DESTINATION lib/${PROJECT_NAME})

# rc: CRSF / Crossfire
set(XCRSF_SRC_LIST
    src/rc/xcrsf/crossfire.cpp src/rc/xcrsf/serial.cpp src/rc/xcrsf/crc.cpp
    src/rc/xcrsf/utils.cpp     src/rc/xcrsf/handler.cpp)
add_library(xcrsf SHARED ${XCRSF_SRC_LIST})
target_link_libraries(xcrsf PRIVATE ${CMAKE_THREAD_LIBS_INIT} atomic)
add_executable(rc_node src/rc/rc_node.cpp)
ament_target_dependencies(rc_node rclcpp std_msgs autoboat_msgs)
target_link_libraries(rc_node xcrsf ${CMAKE_THREAD_LIBS_INIT} ${UDEV_LIB})
install(TARGETS xcrsf ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS rc_node DESTINATION lib/${PROJECT_NAME})

# wind_sensor
add_executable(wind_sensor_node src/wind_sensor/wind_sensor_node.cpp)
ament_target_dependencies(wind_sensor_node rclcpp std_msgs geometry_msgs serial_driver)
target_link_libraries(wind_sensor_node ${UDEV_LIB})
install(TARGETS wind_sensor_node DESTINATION lib/${PROJECT_NAME})
```

### `src/` layout

```
src/
├── utils.hpp                      # shared udev helpers (get_vid_pid_from_device_filepath, ...)
├── motor_controller/
│   ├── vesc_node.{cpp,hpp}
│   └── vesc_protocol/             # buffer, crc, vesc_protocol
├── rc/
│   ├── rc_node.{cpp,hpp}
│   └── xcrsf/                     # crossfire, serial, crc, utils, handler
└── wind_sensor/
    └── wind_sensor_node.{cpp,hpp}
```

### `drivers_cpp` conventions

- All three nodes follow the **`drivers::common::IoContext` + `drivers::serial_driver::SerialDriver` + `std::shared_ptr<SerialPort>`** trio for serial I/O (from the `serial_driver` ROS package).
- `~Node() override` declared (resources to release: serial port, IoContext).
- Protocol logic lives in a separate **SHARED library** (`vesc_protocol`, `xcrsf`) that is also `install`ed to `lib/` (so it ships in the `.deb`).
- Threads linked via `${CMAKE_THREAD_LIBS_INIT} atomic` (xcrsf uses atomics).
- `find_library(UDEV_LIB udev)` for libudev (used by `utils.hpp`).
- `utils.hpp` — shared non-class helpers (udev VID/PID lookup, device-path-from-VID/PID, CPU/RAM stats). Free functions, `namespace fs = std::filesystem;`.

> ⚠️ `drivers_cpp/package.xml` declares only `<depend>autoboat_msgs</depend>` — `serial_driver`, `Threads`, `udev` are used in CMake but NOT declared. Same devcontainer-system-package pattern as `autopilot_cpp`.

### Representative node (`wind_sensor_node.hpp`)

```cpp
class WindSensorPublisher : public rclcpp::Node {
public:
    WindSensorPublisher();
    ~WindSensorPublisher() override;

private:
    drivers::common::IoContext io_ctx;
    drivers::serial_driver::SerialDriver serial_driver;
    std::shared_ptr<drivers::serial_driver::SerialPort> serial_port;

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr apparent_wind_vector_publisher;
    rclcpp::TimerBase::SharedPtr main_loop_timer;
    std::deque<std::pair<double,double>> wind_history;

    void main_loop();
    double sum_integers(int n);
    std::pair<double, double> weighted_average(const std::deque<std::pair<double,double>> &d);
};
```

## Things to avoid

- Editing files under `build/`, `install/`, `log/` — colcon build artifacts.
- Adding a `find_package` without a matching `<depend>` in `package.xml` (or the devcontainer won't have it on a fresh machine).
- Using `auto` for `json` iterator types — be explicit when iterating `nlohmann::json`.
- Forgetting `set__data(...)` for message fields that need to be published with a specific value (ROS 2 default-constructs to zero).
- Storing subscriptions in local variables — they'll be destroyed when the constructor returns. Use the `std::vector<rclcpp::SubscriptionBase::SharedPtr> sub` pattern or member fields.
- Assuming `Position(longitude, latitude)` and `get_latitude_longitude()` agree on order — they don't.
- Changing enum values in `autopilot_utils.hpp` without mirroring in both Python copies (see `python.instructions.md` "Enum sync rule").
- Using `output="screen"` in launch files for high-rate nodes.
