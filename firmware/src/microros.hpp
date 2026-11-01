#ifndef MICROROS_H
#define MICROROS_H

#include "common_libraries.h"
#include <string>
#include <functional>
#include <math.h>

//change these with rewritten libraries
#include "amt22_encoder_library.hpp"
#include "drv8711_stepper_motor_driver_library.hpp"
#include "contactor_driver_library.h"
#include "cmps14_compass.hpp"


// Specific device libraries
#include "hardware/pwm.h"

#include <math.h>


#ifdef __cplusplus
extern "C" {
#endif


#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/empty.h>


#ifdef __cplusplus
}
#endif

static float desired_rudder_angle = 0;
static float desired_rudder_motor_angle = 0;

static float desired_sail_angle = 0;
static float desired_winch_angle = 0;

using namespace std;


//-----------Peripheral Communication Suite-------------
// Encapsulated by structs
// 1. Composite datatypes with some values predefined
// 2. groups of variables that are supposed to be used toegther

//Theseus rudder encoder periheral
struct zero_rudder
{
    inline static rcl_subscription_t zero_rudder_encoder_subscriber;
    inline static string topic = "/zero_rudder_encoder";
    inline static std_msgs__msg__Bool zero_rudder_encoder_msg;
    inline static amt22* encoder = nullptr; // initalized in HAL through systems

    static void zero_rudder_encoder_callback(const void *msg_in);
    static void zero_rudder_create_subscription(rcl_node_t *microros_node);
    static void zero_rudder_add_subscription_to_executor(rclc_executor_t *executor);
};

//Lumpy winch encoder periheral
struct zero_winch
{
    inline static rcl_subscription_t zero_winch_encoder_subscriber;
    inline static std_msgs__msg__Bool zero_winch_encoder_msg;
    inline static string topic = "/zero_winch_encoder";
    inline static amt22* encoder = nullptr; // initalized in HAL through systems

    static void zero_winch_encoder_callback(const void *msg_in);
    static void zero_winch_create_subscription(rcl_node_t *microros_node);
    static void zero_winch_add_subscription_to_executor(rclc_executor_t *executor);
};

// Theseus Peripheral
struct propeller_motor
{
    inline static rcl_subscription_t should_propeller_motor_be_powered_subscriber;
    inline static std_msgs__msg__Bool should_propeller_motor_be_powered_msg;
    inline static string topic = "/should_propeller_motor_be_powered";

    static void should_propeller_motor_be_powered_callback(const void *msg_in);
    static void propeller_motor_create_subscription(rcl_node_t *microros_node);
    static void propeller_motor_add_subscription_to_executor(rclc_executor_t *executor);
};

// both theseus and lumpy have a rudder motor
// for lumpy, angle and motor are the same
struct current_rudder
{
    inline static rcl_publisher_t current_rudder_angle_publisher;
    inline static rcl_publisher_t current_rudder_motor_angle_publisher;
    inline static rcl_subscription_t desired_rudder_angle_subscriber;
    inline static std_msgs__msg__Float32 desired_rudder_angle_msg;
    inline static std_msgs__msg__Float32 current_angle_msg;       // for publishing
    inline static std_msgs__msg__Float32 current_motor_angle_msg; // for publishing
    inline static std_msgs__msg__Float32 current_rudder_motor_angle_msg;

    inline static string angle_publisher_topic = "/current_rudder_angle";
    inline static string motor_angle_publisher_topic = "/current_rudder_motor_angle";
    inline static string angle_subscriber_topic = "/desired_rudder_angle";

    inline static float desired_angle = 0;       // desired rudder angle (state)
    inline static float desired_motor_angle = 0; // desired motor angle (state)

    static void create_current_rudder_motor_angle_publisher(rcl_node_t *microros_node);
    static void create_current_rudder_angle_publisher(rcl_node_t *microros_node);
    static void create_desired_rudder_angle_subscriber(rcl_node_t *microros_node);
    static void add_rudder_angle_to_executor(rclc_executor_t *executor);
    static void desired_rudder_angle_received_callback(const void *msg_in);
};

struct rudder_debug
{
    inline static rcl_publisher_t rudder_debug_publisher;
    inline static string debug_publisher_topic = "/motor_controller_register";
    static void create_rudder_debug_publisher(rcl_node_t *microros_node);
    inline static std_msgs__msg__Int32 current_register_value_msg;
};

// Lumpy peripheral
struct current_sail
{
    inline static rcl_publisher_t current_sail_angle_publisher;
    inline static rcl_publisher_t current_winch_angle_publisher;
    inline static rcl_subscription_t desired_winch_angle_subscriber;
    inline static std_msgs__msg__Float32 desired_winch_angle_msg;
    inline static std_msgs__msg__Float32 current_sail_angle_msg;  // for publishing
    inline static std_msgs__msg__Float32 current_winch_angle_msg; // for publishing

    inline static string sail_angle_publisher_topic = "/current_sail_angle";
    inline static string winch_angle_publisher_topic = "/winch_sail_angle";
    inline static string desired_angle_subscriber_topic = "/desired_sail_angle";

    inline static float desired_sail_angle = 0;  // desired sail angle (state)
    inline static float desired_winch_angle = 0; // desired winch angle (state)

    static void desired_sail_angle_received_callback(const void *msg_in);
    static void create_current_winch_publisher(rcl_node_t *microros_node);
    static void create_current_sail_publisher(rcl_node_t *microros_node);
    static void create_desired_sail_angle_subscriber(rcl_node_t *microros_node);
    static void add_sail_angle_to_executor(rclc_executor_t *executor);
};

struct current_heading
{
    inline static rcl_publisher_t compass_angle_publisher;
    inline static string topic = "/heading";
    inline static std_msgs__msg__Float32 heading_msg; // for publishing

    static void create_heading_publisher(rcl_node_t *microros_node);
};

class Microros
{
public:
    Microros();
    
    void set_cores(rcl_allocator_t* allocator, rclc_support_t* support, rclc_executor_t* executor);

    void initialize_lumpy_peripherals();
    void initialize_theseus_peripherals();
    void initialize_debug();


private:
    rcl_node_t microros_node;
    rcl_allocator_t* allocator;
    rclc_support_t* support;
    rclc_executor_t* executor;
};

#endif