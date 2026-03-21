#include "microros.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#include <rcl/rcl.h>
#include <rclc/rclc.h>

#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/empty.h>
rcl_node_t microros_node;
rmw_qos_profile_t best_effort_qos_profile;

rcl_timer_t application_loop_timer;


std_msgs__msg__Float64 desired_rudder_angle_msg;
std_msgs__msg__Float64 desired_winch_angle_msg;
std_msgs__msg__Float64 current_rudder_angle_msg;
std_msgs__msg__Float64 current_rudder_motor_angle_msg;
std_msgs__msg__Float64 current_sail_angle_msg;
std_msgs__msg__Float64 current_winch_angle_msg;
std_msgs__msg__Float64 compass_angle_msg;

std_msgs__msg__Empty empty_request_msg;
std_msgs__msg__Empty empty_response_msg;

std_msgs__msg__Float64 test_msg;

void application_loop(rcl_timer_t * timer, int64_t last_call_time);

#ifdef __cplusplus
}
#endif

//-----------------RUDDER---------------------
void zero_rudder::zero_rudder_encoder_callback(const void *msg_in)
{
    const std_msgs__msg__Bool *zero_msg = (const std_msgs__msg__Bool *)msg_in;

    if (zero_msg->data)
    { // If message is true, zero the encoder
        // zero_encoder_value(zero_rudder::encoder);
        zero_rudder::encoder->zero_encoder_value();
    }
}

void zero_rudder::zero_rudder_create_subscription(rcl_node_t *microros_node)
{
    rclc_subscription_init_default(
        &zero_rudder::zero_rudder_encoder_subscriber,
        microros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        zero_rudder::topic.c_str());
}

void zero_rudder::zero_rudder_add_subscription_to_executor(rclc_executor_t *executor)
{
    rclc_executor_add_subscription(
        executor,
        &zero_rudder::zero_rudder_encoder_subscriber,
        &zero_rudder::zero_rudder_encoder_msg,
        &zero_rudder::zero_rudder_encoder_callback,
        ON_NEW_DATA);
}

//------------------WINCH---------------------------
void zero_winch::zero_winch_encoder_callback(const void *msg_in)
{
    const std_msgs__msg__Bool *zero_msg = (const std_msgs__msg__Bool *)msg_in;

    if (zero_msg->data)
    { // If message is true, zero the encoder
        // zero_encoder_value(zero_winch::encoder);
        zero_winch::encoder->zero_encoder_value();
    }
}

void zero_winch::zero_winch_create_subscription(rcl_node_t *microros_node)
{
    rclc_subscription_init_default(
        &zero_winch::zero_winch_encoder_subscriber,
        microros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        zero_winch::topic.c_str());
}

void zero_winch::zero_winch_add_subscription_to_executor(rclc_executor_t *executor)
{
    rclc_executor_add_subscription(
        executor,
        &zero_winch::zero_winch_encoder_subscriber,
        &zero_winch::zero_winch_encoder_msg,
        &zero_winch::zero_winch_encoder_callback,
        ON_NEW_DATA);
}

//---------------------Propeller-------------------------
void propeller_motor::should_propeller_motor_be_powered_callback(const void *msg_in)
{
    const std_msgs__msg__Bool *should_propeller_motor_be_powered_msg = (const std_msgs__msg__Bool *)msg_in;

    if (should_propeller_motor_be_powered_msg->data)
        close_contactor(CONTACTOR_DRIVER_IN_A_PIN);
    else
        open_contactor(CONTACTOR_DRIVER_IN_A_PIN);
}

void propeller_motor::propeller_motor_create_subscription(rcl_node_t *microros_node)
{
    rclc_subscription_init_default(
        &propeller_motor::should_propeller_motor_be_powered_subscriber,
        microros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        propeller_motor::topic.c_str());
}

void propeller_motor::propeller_motor_add_subscription_to_executor(rclc_executor_t *executor)
{
    rclc_executor_add_subscription(
        executor,
        &propeller_motor::should_propeller_motor_be_powered_subscriber,
        &propeller_motor::should_propeller_motor_be_powered_msg,
        &propeller_motor::should_propeller_motor_be_powered_callback,
        ON_NEW_DATA);
}

//---------------------Current Rudder-------------------------
void current_rudder::create_current_rudder_motor_angle_publisher(rcl_node_t *microros_node)
{
    rclc_publisher_init_default(
        &current_rudder::current_rudder_motor_angle_publisher,
        microros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        current_rudder::motor_angle_publisher_topic.c_str());
}

void current_rudder::create_current_rudder_angle_publisher(rcl_node_t *microros_node)
{
    rclc_publisher_init_default(
        &current_rudder::current_rudder_angle_publisher,
        microros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
        current_rudder::angle_publisher_topic.c_str());
}

void current_rudder::create_desired_rudder_angle_subscriber(rcl_node_t *microros_node)
{
    rclc_subscription_init_best_effort(
        &current_rudder::desired_rudder_angle_subscriber,
        microros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        current_rudder::angle_subscriber_topic.c_str());
}

void current_rudder::add_rudder_angle_to_executor(rclc_executor_t *executor)
{
    rclc_executor_add_subscription(
        executor,
        &current_rudder::desired_rudder_angle_subscriber,
        &current_rudder::desired_rudder_angle_msg,
        &current_rudder::desired_rudder_angle_received_callback,
        ON_NEW_DATA);
}

void current_rudder::desired_rudder_angle_received_callback(const void *msg_in)
{
    const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msg_in;
    current_rudder::desired_angle = -(msg->data);

    if (current_rudder::desired_angle > MAX_RUDDER_ANGLE)
        current_rudder::desired_angle = MAX_RUDDER_ANGLE;
    if (current_rudder::desired_angle < MIN_RUDDER_ANGLE)
        current_rudder::desired_angle = MIN_RUDDER_ANGLE;

    current_rudder::desired_motor_angle = get_motor_angle_from_rudder_angle(current_rudder::desired_angle);
}

//---------------------Current Sail-------------------------
void current_sail::create_current_sail_publisher(rcl_node_t *microros_node)
{
    rclc_publisher_init_default(
        &current_sail::current_sail_angle_publisher,
        microros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        current_sail::sail_angle_publisher_topic.c_str());
}

void current_sail::create_current_winch_publisher(rcl_node_t *microros_node)
{
    rclc_publisher_init_default(
        &current_sail::current_winch_angle_publisher,
        microros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        current_sail::winch_angle_publisher_topic.c_str());
}

void current_sail::create_desired_sail_angle_subscriber(rcl_node_t *microros_node)
{
    rclc_subscription_init_best_effort(
        &current_sail::desired_winch_angle_subscriber,
        microros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        current_sail::desired_angle_subscriber_topic.c_str());
}

void current_sail::add_sail_angle_to_executor(rclc_executor_t *executor)
{
    rclc_executor_add_subscription(
        executor,
        &current_sail::desired_winch_angle_subscriber,
        &current_sail::desired_winch_angle_msg,
        &current_sail::desired_sail_angle_received_callback,
        ON_NEW_DATA);
}

void current_sail::desired_sail_angle_received_callback(const void *msg_in)
{
    const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msg_in;
    current_sail::desired_sail_angle = msg->data;

    if (current_sail::desired_sail_angle > MAX_SAIL_ANGLE)
        current_sail::desired_sail_angle = MAX_SAIL_ANGLE;
    if (current_sail::desired_sail_angle < MIN_SAIL_ANGLE)
        current_sail::desired_sail_angle = MIN_SAIL_ANGLE;

    current_sail::desired_winch_angle = get_winch_angle_from_sail_angle(current_sail::desired_sail_angle);
}

//---------------------Current Heading-------------------------
void current_heading::create_heading_publisher(rcl_node_t *microros_node)
{
    rclc_publisher_init_default(
        &current_heading::compass_angle_publisher,
        microros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        current_heading::topic.c_str());
}

//----------------MICROROS-CLASS----------------------------------

Microros::Microros(rcl_allocator_t* allocator, rclc_support_t* support, rclc_executor_t* executor)
{
    this->allocator = allocator;
    this->support = support;
    this->executor = executor;
    rclc_node_init_default(&this->microros_node, "microros", "", this->support);
}

void Microros::initialize_lumpy_peripherals()
{
    // Zero rudder encoder
    zero_rudder::zero_rudder_create_subscription(&microros_node);
    zero_rudder::zero_rudder_add_subscription_to_executor(executor);

    // Zero winch encoder
    zero_winch::zero_winch_create_subscription(&microros_node);
    zero_winch::zero_winch_add_subscription_to_executor(executor);

    // Current rudder (publishers + subscriber)
    current_rudder::create_current_rudder_angle_publisher(&microros_node);
    current_rudder::create_current_rudder_motor_angle_publisher(&microros_node);
    current_rudder::create_desired_rudder_angle_subscriber(&microros_node);
    current_rudder::add_rudder_angle_to_executor(executor);

    // Current sail (publishers + subscriber)
    current_sail::create_current_sail_publisher(&microros_node);
    current_sail::create_current_winch_publisher(&microros_node);
    current_sail::create_desired_sail_angle_subscriber(&microros_node);
    current_sail::add_sail_angle_to_executor(executor);

    // Heading publisher
    current_heading::create_heading_publisher(&microros_node);
}

void Microros::initialize_theseus_peripherals()
{
    // Zero rudder encoder
    zero_rudder::zero_rudder_create_subscription(&microros_node);
    zero_rudder::zero_rudder_add_subscription_to_executor(executor);

    // Propeller motor contactor control
    propeller_motor::propeller_motor_create_subscription(&microros_node);
    propeller_motor::propeller_motor_add_subscription_to_executor(executor);

    // Current rudder (publishers + subscriber)
    current_rudder::create_current_rudder_angle_publisher(&microros_node);
    current_rudder::create_current_rudder_motor_angle_publisher(&microros_node);
    current_rudder::create_desired_rudder_angle_subscriber(&microros_node);
    current_rudder::add_rudder_angle_to_executor(executor);

    // Heading publisher
    current_heading::create_heading_publisher(&microros_node);

    RCCHECK(rclc_timer_init_default(&application_loop_timer, support, RCL_MS_TO_NS(1), application_loop));
    RCCHECK(rclc_executor_add_timer(executor, &application_loop_timer));

    
    // -----------------------------------------------------
    //  INITIALIZE SPI AND I2C BUSSES
    // -----------------------------------------------------
    spi_init(SPI_PORT, 500 * 1000);
    gpio_set_function(SCLK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MISO_PIN, GPIO_FUNC_SPI);

    // i2c_init(I2C_PORT, 100 * 1000);
    // gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    // gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    // gpio_pull_up(SDA_PIN);
    // gpio_pull_up(SCL_PIN);
    
    // rudderEncoder = amt22::amt22(RUDDER_ENCODER_CS_PIN, SPI_PORT);

    

    // AMT22_init(&rudderEncoder, RUDDER_ENCODER_CS_PIN, SPI_PORT);
    // drv8711_init(&rudderStepperMotorDriver, SPI_PORT, RUDDER_MOTOR_CS_PIN, RUDDER_MOTOR_SLEEP_PIN, AutoMixed, RUDDER_MICROSTEP, MAX_RUDDER_CURRENT);

    // cmps14_init(&compass, I2C_PORT, 0x60);
    // initialize_contactor_driver(CONTACTOR_DRIVER_SEL0_PIN, CONTACTOR_DRIVER_PWM_PIN, CONTACTOR_DRIVER_IN_A_PIN, CONTACTOR_DRIVER_IN_B_PIN);

    // -----------------------------------------------------
    // INITIALIZE DEFAULT CALLBACK VARIABLE VALUES
    // -----------------------------------------------------
    desired_rudder_angle_msg.data = 0.0;
    current_rudder_angle_msg.data = 0.0;
    compass_angle_msg.data = 0.0;


    gpio_init(14);
    gpio_set_dir(14, GPIO_OUT);

    gpio_init(15);
    gpio_set_dir(15, GPIO_OUT);

    gpio_init(16);
    gpio_set_dir(16, GPIO_OUT);

    // gpio_pull_up(14);
    // gpio_pull_up(15);
    // gpio_pull_up(16);

                gpio_put(14,0);
            gpio_put(15,0);
            gpio_put(16,1);



}

void application_loop(rcl_timer_t * timer, int64_t last_call_time)
{

    (void) timer;
    (void) last_call_time;

    // -----------------------------------------------------
    // RUDDER CLOSED LOOP CONTROl
    // -----------------------------------------------------
    // float current_rudder_motor_angle = get_motor_angle(&rudderEncoder) + RUDDER_ANGLE_OFFSET; // motor_angle % 360
    // if (current_rudder_motor_angle >= 180)
    // {
    //     current_rudder_motor_angle -= 360;
    // }

    // float current_rudder_angle = get_rudder_angle_from_motor_angle(current_rudder_motor_angle);
    // float rudder_error = current_rudder_angle - desired_rudder_angle;

    // int number_of_steps_rudder = 0;
    // bool rudder_step_enabled = false;

    // if (abs(rudder_error) > ACCEPTABLE_RUDDER_ERROR)
    // {
    //     rudder_step_enabled = true;

    //     if (((int)rudder_error % 360) > 0 && ((int)rudder_error % 360) < 180)
    //         // drv8711_setDirection(&rudderStepperMotorDriver, COUNTER_CLOCKWISE);

    //     else
    //         drv8711_setDirection(&rudderStepperMotorDriver, CLOCKWISE);

    //     // number_of_steps_rudder = RUDDER_GAIN * abs(rudder_error) + RUDDER_GAIN_Q * pow(abs(rudder_error), 2);
    //     number_of_steps_rudder = (int)(abs(rudder_error) * RUDDER_GAIN / MAX_RUDDER_ERROR);

    //     if (number_of_steps_rudder > RUDDER_NUMBER_OF_STEPS_TO_CLIP_AT)
    //     {
    //         number_of_steps_rudder = RUDDER_NUMBER_OF_STEPS_TO_CLIP_AT;
    //     }
    // }

    // // -----------------------------------------------------
    // // SAIL CLOSED LOOP CONTROl
    // // -----------------------------------------------------
    // int number_of_steps_winch = 0;
    // bool winch_step_enabled = false;

    // float angle = get_motor_angle(&rudderEncoder);

    // cs_select(&rudderEncoder);

    // float angle = rudderEncoder.get_motor_angle();
    // rudderEncoder.putLow();


    // current_rudder_angle_msg.data = angle;


    // RCCHECK(rcl_publish(&current_rudder::current_rudder_angle_publisher, &current_rudder_angle_msg, NULL));

// #if BOAT_MODE == Lumpy
//     float current_winch_angle = get_motor_angle(&winchEncoder) + WINCH_ANGLE_OFFSET + 360 * get_turn_count(&winchEncoder);
//     float current_sail_angle = get_sail_angle_from_winch_angle(current_winch_angle);
//     float winch_error = desired_winch_angle - current_winch_angle;

//     // check for absurd errors, and if there is an absurd error, do nothing
//     if (abs(winch_error) > 1000000 || winch_error != winch_error)
//         winch_error = 0;

//     if (abs(winch_error) > ACCEPTABLE_WINCH_ERROR)
//     {
//         winch_step_enabled = true;

//         if (winch_error > 0)
//         {
//             drv8711_setDirection(&winchStepperMotorDriver, CLOCKWISE);
//         }
//         else
//         {
//             drv8711_setDirection(&winchStepperMotorDriver, COUNTER_CLOCKWISE);
//         }

//         // number of steps is some linear function that maps the error of the rudder to a number of steps we want to take per loop.
//         // This ends up cooresponding to the speed of the rudder. The higher the rudder_error, the higher the speed of the rudder will be
//         number_of_steps_winch = (int)(abs(winch_error) * WINCH_GAIN / MAX_WINCH_ERROR);

//         if (number_of_steps_winch > WINCH_NUMBER_OF_STEPS_TO_CLIP_AT)
//         {
//             number_of_steps_winch = WINCH_NUMBER_OF_STEPS_TO_CLIP_AT;
//         }
//     }

//     // -----------------------------------------------------
//     // CLOSED LOOP CONTROL STEPPING
//     // -----------------------------------------------------
//     int max_steps = number_of_steps_rudder;
// #if BOAT_MODE == Lumpy
//     if (number_of_steps_winch > max_steps)
//         max_steps = number_of_steps_winch;
// #endif

//     for (int i = 0; i < max_steps; i++)
//     {
//         if (rudder_step_enabled && i < number_of_steps_rudder)
//             drv8711_step(&rudderStepperMotorDriver);

// #if BOAT_MODE == Lumpy
//         if (winch_step_enabled && i < number_of_steps_winch)
//             drv8711_step(&winchStepperMotorDriver);
// #endif

//         sleep_us(MIN_TIME_BETWEEN_MOTOR_STEPS_MICROSECONDS);
//     }

//     current_sail_angle_msg.data = current_sail_angle;
//     current_winch_angle_msg.data = current_winch_angle;

//     rcl_publish(&current_winch_angle_publisher, &current_winch_angle_msg, NULL);
//     rcl_publish(&current_sail_angle_publisher, &current_sail_angle_msg, NULL);

//     // counter clockwise from true east
//     compass_angle_msg.data = fmod((-cmps14_getBearing(&compass) / 10.0 + COMPASS_OFFSET + 360), 360.0);
//     current_rudder_angle_msg.data = current_rudder_angle;
//     current_rudder_motor_angle_msg.data = current_rudder_motor_angle;

//     test_msg.data = rudder_error;

//     rcl_publish(&test_publisher, &test_msg, NULL);
//     rcl_publish(&current_rudder_motor_angle_publisher, &current_rudder_motor_angle_msg, NULL);
//     rcl_publish(&current_rudder_angle_publisher, &current_rudder_angle_msg, NULL);
//     rcl_publish(&compass_angle_publisher, &compass_angle_msg, NULL);

// #endif
}

