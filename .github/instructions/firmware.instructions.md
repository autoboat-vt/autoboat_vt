---
description: "Use when writing or editing RP2040 firmware under firmware/. Covers two-mode CMakeLists (host CLI unit tests with Catch2 vs Pico SDK build, $ENV{CMAKE_CURRENT_SOURCE_DIR} bug, PICO_UART_ENABLE_CRLF_SUPPORT=0), main.cpp outer while(true) re-establishes transport + rmw_uros_set_custom_transport + ping loops, microros.hpp per-peripheral struct pattern (inline static state + static methods, NOT class-per-peripheral), Microros orchestrator class, publisher/subscriber creation idioms (rclc_subscription_init_default vs _best_effort), callback C-style cast, globals (static sensor instances), #if BOAT_MODE calibration polynomials, device classes (HAL, spi_device, amt22_encoder, i2c_device, cmps14_compass, systems, boat with boat_type enum), MAGNETIC_DECLINATION/HEADING_OFFSET constants."
applyTo: "firmware/**"
---

# Firmware Conventions

Firmware runs on the **RP2040 (Raspberry Pi Pico)** under the Pico SDK + micro-ROS. C++17. Built via a two-mode `CMakeLists.txt`: host CLI unit tests vs Pico SDK UF2 build. Talks to ROS 2 via a serial micro-ROS agent (`micro_ros_agent serial --dev /dev/pico -b 115200`, launched from `motorboat.launch.py`).

## `CMakeLists.txt` — two-mode build

File: `firmware/CMakeLists.txt`. Builds either host-side unit tests (CLI) or Pico SDK firmware (UF2), gated on `BUILD_UNIT_TESTS` option.

```cmake
cmake_minimum_required(VERSION 3.13)
project(firmware C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

option(BUILD_UNIT_TESTS "Build host-side unit tests instead of Pico firmware" OFF)

if(BUILD_UNIT_TESTS)
    # Host CLI build: ament_cmake + Catch2 for unit tests
    find_package(ament_cmake REQUIRED)
    find_package(Catch2 REQUIRED)

    add_executable(firmware_test
        src/main.cpp
        src/microros.cpp
        # ... device sources ...
    )
    target_link_libraries(firmware_test Catch2::Catch2)
    target_compile_definitions(firmware_test PRIVATE HOST_TEST=1)
    ament_package()
else()
    # Pico SDK build
    include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
    pico_sdk_init()

    link_directories($ENV{PICO_MICROROS_SDK_PATH}/libmicroros)

    add_executable(firmware
        src/main.cpp
        src/microros.cpp
        # ... device sources ...
    )
    target_link_libraries(firmware
        pico_stdlib
        hardware_spi
        hardware_i2c
        hardware_pwm
        libmicroros
    )
    target_compile_definitions(firmware PRIVATE
        PICO_UART_ENABLE_CRLF_SUPPORT=0
    )
    pico_enable_stdio_usb(firmware 1)
    pico_enable_stdio_uart(firmware 0)
    pico_add_extra_outputs(firmware)          # produces firmware.uf2
endif()
```

### Conventions

- **C++17** (NOT C++23 like `autopilot_cpp` — Pico SDK limitation).
- `PICO_UART_ENABLE_CRLF_SUPPORT=0` — the micro-ROS transport needs raw bytes, no CR/LF translation.
- `pico_enable_stdio_usb(firmware 1)` / `pico_enable_stdio_uart(firmware 0)` — log output goes to USB CDC, not UART (UART is used by micro-ROS).
- `pico_add_extra_outputs(firmware)` produces `firmware.uf2` (drag-flashable) + map/bin.
- micro-ROS static library lives at `$ENV{PICO_MICROROS_SDK_PATH}/libmicroros` — prebuilt per-board.
- `link_directories($ENV{PICO_MICROROS_SDK_PATH}/libmicroros)` BEFORE `add_executable`.
- Host build uses `ament_cmake` + Catch2 and defines `HOST_TEST=1` so device classes can stub hardware calls.

> ⚠️ **Bug:** `target_include_directories(firmware PRIVATE $ENV{CMAKE_CURRENT_SOURCE_DIR}/include)` appears in the file, but `$ENV{CMAKE_CURRENT_SOURCE_DIR}` expands empty (it's a CMake variable, not an env var). Use `${CMAKE_CURRENT_SOURCE_DIR}` instead. In practice the include resolves via relative paths / other include dirs.

## `main.cpp` — outer reconnect loop

File: `firmware/src/main.cpp`. The outer `while (true)` re-establishes the micro-ROS transport when the agent is lost; the inner loop runs the boat's `Systems::check_microros()` tick.

```cpp
#include "microros.hpp"
#include "systems.hpp"

int main() {
    stdio_init_all();
    // ... hardware init (SPI, I2C, PWM) ...

    while (true) {
        // Re-establish transport on agent loss
        rmw_uros_set_custom_transport(
            true,
            nullptr,
            pico_serial_open,
            pico_serial_close,
            pico_serial_write,
            pico_serial_read
        );

        // Wait for agent — 1s timeout, 120 attempts (~2 min)
        rcl_ret_t ret = rmw_uros_ping_agent(1000, 120);
        if (ret != RCL_RET_OK) {
            continue;   // no agent; loop and try again
        }

        Systems system = Systems(BOAT_TYPE::THESEUS);   // boat-specific peripheral set
        if (!system.initialize_microros()) {
            continue;
        }

        while (true) {
            // Inner loop — ping agent (1s, 5 attempts)
            rcl_ret_t ping_ret = rmw_uros_ping_agent(1000, 5);
            if (ping_ret != RCL_RET_OK) {
                break;   // agent lost → outer loop re-establishes
            }
            system.check_microros();   // spin executor, run callbacks
            system.loop();             // read sensors, publish
        }
        system.cleanup();
    }
    return 0;
}
```

### Pattern notes

- **Outer `while (true)` = re-establish transport on agent loss.** The Pico never gives up; if the agent (running on the companion Pi) restarts, the Pico reconnects.
- `rmw_uros_set_custom_transport(true, nullptr, open, close, write, read)` — `true` = reliable (vs best-effort). The four `pico_serial_*` functions live in `microros.cpp` and wrap Pico SDK UART calls.
- **Two ping loops with different parameters:** outer reconnect uses `ping_agent(1000, 120)` (1s × 120 = 2 min patience); inner steady-state uses `ping_agent(1000, 5)` (1s × 5 = 5s patience before declaring agent lost).
- `Systems(BOAT_TYPE::THESEUS)` — the boat type enum selects which peripherals are initialized. `LUMPY` is the other variant.
- `system.initialize_microros()` chains all `create_*` + `add_*_to_executor` calls; returns false on any failure.
- `system.check_microros()` spins the executor once (non-blocking); `system.loop()` does sensor I/O + publishing.

## `microros.hpp/cpp` — per-peripheral struct pattern

File: `firmware/src/microros.hpp`. **Critical pattern:** peripherals are NOT one-class-each. Instead, each peripheral is a `struct` of `inline static` state (publisher, subscription, msg, topic name, desired angle) + a set of `static` methods (`create_*`, `add_*_to_executor`, `*_callback`). The `Microros` orchestrator class holds the allocator/support/executor and chains the per-peripheral setup.

```cpp
// microros.hpp
#include <rcl/rcl.h>
#include <rcl/executor.h>
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include "autoboat_msgs/msg/rc_data.hpp"

struct RudderPeripheral {
    inline static rcl_publisher_t   angle_publisher;
    inline static rcl_subscription_t desired_angle_subscription;
    inline static std_msgs__msg__Float32 angle_msg;
    inline static std_msgs__msg__Float32 desired_angle_msg;
    inline static const char* const topic = "/current_rudder_angle";
    inline static const char* const desired_topic = "/desired_rudder_angle";
    inline static float desired_angle = 0.0f;

    static rcl_ret_t create(rcl_node_t* node) {
        rcl_ret_t rc = RCL_RET_OK;
        rmw_qos_profile_t qos = rmw_qos_profile_default;
        rc += rclc_publisher_init_default(&angle_publisher, node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), topic);
        rc += rclc_subscription_init_default(&desired_angle_subscription, node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), desired_topic);
        return rc;
    }
    static rcl_ret_t add_to_executor(rclc_executor_t* executor) {
        return rclc_executor_add_subscription(executor, &desired_angle_subscription,
            &desired_angle_msg, &desired_angle_callback, ON_NEW_DATA);
    }
    static void desired_angle_callback(const void* msgin) {
        const auto* msg = static_cast<const std_msgs__msg__Float32*>(msgin);
        // Clamp to hardware limits
        desired_angle = std::clamp(msg->data, MIN_RUDDER_ANGLE, MAX_RUDDER_ANGLE);
    }
    static rcl_ret_t publish(float angle) {
        angle_msg.data = angle;
        return rcl_publish(&angle_publisher, &angle_msg, nullptr);
    }
};

// ... similar structs for Winch, RC, etc. ...

class Microros {
public:
    inline static rcl_allocator_t      allocator;
    inline static rclc_support_t       support;
    inline static rcl_node_t           node;
    inline static rclc_executor_t      executor;

    bool initialize_theseus_peripherals() {
        // chain create + add_to_executor for each peripheral
        if (RudderPeripheral::create(&node) != RCL_RET_OK) return false;
        if (WinchPeripheral::create(&node)   != RCL_RET_OK) return false;
        if (RCPeripheral::create(&node)      != RCL_RET_OK) return false;
        if (RudderPeripheral::add_to_executor(&executor) != RCL_RET_OK) return false;
        if (WinchPeripheral::add_to_executor(&executor)   != RCL_RET_OK) return false;
        if (RCPeripheral::add_to_executor(&executor)      != RCL_RET_OK) return false;
        return true;
    }
};
```

### Pattern notes

- **Per-peripheral struct, not class-per-peripheral.** Each peripheral is a `struct` with `inline static` state + `static` methods. This keeps all micro-ROS handles at static storage (micro-ROS on RP2040 requires static allocation — no heap).
- `inline static` (C++17) lets the declaration live in the header without needing a separate `.cpp` definition.
- **Callback signature is C-style:** `void callback(const void* msgin)` → cast to the concrete type inside. This is the micro-ROS executor contract.
- **Clamp in the callback** — hardware safety. `std::clamp(msg->data, MIN_RUDDER_ANGLE, MAX_RUDDER_ANGLE)` before storing.
- `Microros` orchestrator holds the shared allocator/support/node/executor (also `inline static`) and chains `create` + `add_to_executor` for every peripheral in `initialize_theseus_peripherals()`.

### Publisher/subscriber creation idioms

```cpp
// Reliable (default) — for low-rate command/status topics
rclc_publisher_init_default(&pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/current_rudder_angle");
rclc_subscription_init_default(&sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/desired_rudder_angle");

// Best-effort — for high-rate sensor topics (avoids retransmit overhead)
rclc_subscription_init_best_effort(&sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(autoboat_msgs, msg, RCData), "/rc_data");
```

Use `_best_effort` for high-rate sensor data where dropped samples are acceptable (RC data, IMU). Use `_default` (reliable) for commands and low-rate status.

## Globals — static sensor instances

File: `firmware/src/main.cpp` (or a globals header). Sensors are `static` instances at file scope, shared between `main` and the `Systems` class:

```cpp
#include "amt22_encoder.hpp"
#include "cmps14_compass.hpp"

static amt22_encoder rudderEncoder(spi0, CS_RUDDER_PIN);
static amt22_encoder winchEncoder(spi0, CS_WINCH_PIN);
static cmps14_compass compass(i2c0, COMPASS_ADDR);
```

`static` at file scope = single instance, initialized once at startup, shared across functions. The `Systems` class references these by extern declaration.

## `#if BOAT_MODE` calibration polynomials

File: `firmware/src/...`. Boat-specific calibration is gated on a `BOAT_MODE` macro. `THESEUS` and `LUMPY` have different calibration constants (sensor offsets, polynomial coefficients for rudder/winch angle → voltage).

```cpp
#if BOAT_MODE == THESEUS
    #define RUDDER_POLY_A  0.0123f
    #define RUDDER_POLY_B  -0.4567f
    // ...
#elif BOAT_MODE == LUMPY
    #define RUDDER_POLY_A  0.0987f
    #define RUDDER_POLY_B  -0.6543f
    // ...
#endif
```

> ⚠️ These macros appear possibly dead/stale — verify the boat you're targeting before relying on them.

## Constants

```cpp
#define MAGNETIC_DECLINATION  -8.5f    // Blacksburg, VA
#define HEADING_OFFSET        -26.0f   // per-boat compass mounting offset
#define MIN_RUDDER_ANGLE      -30.0f
#define MAX_RUDDER_ANGLE       30.0f
```

`MAGNETIC_DECLINATION = -8.5` (VT location). `HEADING_OFFSET = -26` is a per-boat compass mounting offset (different per hull).

## Device classes

File: `firmware/src/devices/`. Class hierarchy wrapping Pico SDK hardware APIs.

### `HAL.hpp` — hardware abstraction base

```cpp
class HAL {
public:
    virtual void init() = 0;
    virtual ~HAL() = default;
};
```

### `spi_device` / `amt22_encoder` / `i2c_device` / `cmps14_compass`

```cpp
class spi_device : public HAL {
protected:
    spi_inst_t* spi;
    uint cs_pin;
public:
    spi_device(spi_inst_t* spi_, uint cs_pin_);
    void init() override;
    void read(uint8_t* buf, size_t len);
    void write(const uint8_t* buf, size_t len);
};

class amt22_encoder : public spi_device {     // AMT22 absolute encoder
public:
    amt22_encoder(spi_inst_t* spi_, uint cs_pin_);
    float read_angle();                        // returns degrees, 0-360
};

class i2c_device : public HAL {
protected:
    i2c_inst_t* i2c;
    uint8_t address;
public:
    i2c_device(i2c_inst_t* i2c_, uint8_t address_);
    void init() override;
};

class cmps14_compass : public i2c_device {    // CMPS14 compass + tilt
public:
    cmps14_compass(i2c_inst_t* i2c_, uint8_t address_);
    float read_heading();                      // returns degrees, 0-360
};
```

### `systems.hpp` / `boat.hpp` — boat type enum

```cpp
enum class BOAT_TYPE { LUMPY, THESEUS };

class Systems {
private:
    BOAT_TYPE boat_type;
    // ... peripheral state ...
public:
    Systems(BOAT_TYPE type);
    bool initialize_microros();
    void check_microros();     // spin executor
    void loop();               // sensor I/O + publish
    void cleanup();
};
```

`Systems(BOAT_TYPE type)` selects which peripherals exist on this hull. `THESEUS` is the motorboat; `LUMPY` is (historically) the sailboat.

## Things to avoid

- Editing files under `build/`, `install/`, `log/` — build artifacts.
- Using `new`/`malloc` for micro-ROS handles — RP2040 micro-ROS requires **static allocation**. Use `inline static` in structs.
- Using C++20/23 features — Pico SDK + micro-ROS toolchain is C++17.
- Forgetting `PICO_UART_ENABLE_CRLF_SUPPORT=0` — CR/LF translation corrupts micro-ROS frames.
- Forgetting to clamp actuator commands in callbacks — hardware safety.
- Relying on `$ENV{CMAKE_CURRENT_SOURCE_DIR}` — it's a CMake variable, use `${CMAKE_CURRENT_SOURCE_DIR}`.
- Using `rclc_subscription_init_default` (reliable) for high-rate sensor data — use `_best_effort`.
- Declaring a peripheral as a class with non-static members — micro-ROS handles must be static.
- Changing `MAGNETIC_DECLINATION` or `HEADING_OFFSET` without confirming the target boat's mounting.
- Assuming `BOAT_MODE` calibration macros are current — verify before use (possibly stale).
