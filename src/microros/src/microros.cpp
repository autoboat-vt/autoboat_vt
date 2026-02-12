#include "microros.hpp"

//-----------------RUDDER---------------------
void zero_rudder::zero_rudder_encoder_callback(const void *msg_in)
{
    const std_msgs__msg__Bool *zero_msg = (const std_msgs__msg__Bool *)msg_in;

    if (zero_msg->data)
    { // If message is true, zero the encoder
        zero_encoder_value(zero_rudder::encoder);
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
        zero_encoder_value(zero_winch::encoder);
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
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
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
}
