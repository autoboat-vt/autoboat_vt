#include "main_microros_node.h"

rcl_node_t  microros_node;
rmw_qos_profile_t best_effort_qos_profile;

rcl_subscription_t is_propeller_motor_enabled_subscriber;
rcl_subscription_t desired_rudder_angle_subscriber;
rcl_subscription_t desired_winch_angle_subscriber;
rcl_subscription_t zero_rudder_encoder_subscriber;
rcl_subscription_t zero_winch_encoder_subscriber;
rcl_subscription_t pwm_motor_subscriber;
rcl_publisher_t    current_rudder_angle_publisher;
rcl_publisher_t    current_rudder_motor_angle_publisher;
rcl_publisher_t    current_sail_angle_publisher;
rcl_publisher_t    current_winch_angle_publisher;
rcl_publisher_t    compass_angle_publisher;
rcl_timer_t        application_loop_timer;

std_msgs__msg__Bool           is_propeller_motor_enabled_msg;  
std_msgs__msg__Bool           zero_rudder_encoder_msg;
std_msgs__msg__Bool           zero_winch_encoder_msg;
std_msgs__msg__Float32        compass_angle_msg;
std_msgs__msg__Float32        current_rudder_motor_angle_msg;
std_msgs__msg__Float32        current_winch_angle_msg;
std_msgs__msg__Float32        current_rudder_angle_msg;
std_msgs__msg__Float32        current_sail_angle_msg;
std_msgs__msg__Float32        desired_rudder_angle_msg;
std_msgs__msg__Float32        desired_winch_angle_msg;
std_msgs__msg__Float32        pwm_motor_msg;
std_srvs__srv__Empty_Request  empty_request_msg;
std_srvs__srv__Empty_Response empty_response_msg;

static drv8711 rudderStepperMotorDriver;
static drv8711 winchStepperMotorDriver;
static amt22   rudderEncoder;
static amt22   winchEncoder;
static cmps14  compass;


static float desired_rudder_angle = 0;
static float desired_rudder_motor_angle = 0;

static float desired_sail_angle = 0;
static float desired_winch_angle  = 0;

void application_init(rcl_allocator_t *allocator, rclc_support_t *support, rclc_executor_t *executor) {

    // -----------------------------------------------------
    // INITIALIZE THE MICROROS NODE
    // -----------------------------------------------------
    RCCHECK(rclc_node_init_default(&microros_node, "microros", "", support));


    // -----------------------------------------------------
    // INITIALIZE ZEROING THE RUDDER AND WINCH ENCODERS SUBSCRIBERS
    // -----------------------------------------------------
    RCCHECK(rclc_subscription_init_default(&zero_rudder_encoder_subscriber, &microros_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/zero_rudder_encoder"));
    RCCHECK(rclc_executor_add_subscription(executor, &zero_rudder_encoder_subscriber, &zero_rudder_encoder_msg, &zero_rudder_encoder_callback, ON_NEW_DATA));
    #if BOAT_MODE == Lumpy
    RCCHECK(rclc_subscription_init_default(&zero_winch_encoder_subscriber, &microros_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/zero_winch_encoder"));
    RCCHECK(rclc_executor_add_subscription(executor, &zero_rudder_winch_subscriber, &zero_winch_encoder_msg, &zero_winch_encoder_callback, ON_NEW_DATA));
    #endif

    // -----------------------------------------------------
    // INITIALIZE THE PROPELLER MOTOR ENABLE SUBSCRIBER (ONLY FOR THESEUS)
    // -----------------------------------------------------
    #if BOAT_MODE == Theseus
    RCCHECK(rclc_subscription_init_default(&is_propeller_motor_enabled_subscriber, &microros_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/is_propeller_motor_enabled"));
    RCCHECK(rclc_executor_add_subscription(executor, &is_propeller_motor_enabled_subscriber, &is_propeller_motor_enabled_msg, &is_propeller_motor_enabled_callback, ON_NEW_DATA));
    RCCHECK(rclc_subscription_init_default(&pwm_motor_subscriber, &microros_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/is_propeller_motor_enabled"));
    RCCHECK(rclc_executor_add_subscription(executor, &pwm_motor_subscriber, &pwm_motor_msg, &pwm_motor_callback, ON_NEW_DATA));
    #endif
    

    // -----------------------------------------------------
    // INITIALIZE THE CURRENT RUDDER PUBLISHERS AND SUBSCRIBERS
    // -----------------------------------------------------
    RCCHECK(rclc_publisher_init_default(&current_rudder_angle_publisher, &microros_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/current_rudder_angle"));
    RCCHECK(rclc_publisher_init_default(&current_rudder_motor_angle_publisher, &microros_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/current_rudder_motor_angle"));
    RCCHECK(rclc_subscription_init_best_effort(&desired_rudder_angle_subscriber, &microros_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/actions/rudder_angle"));       
    RCCHECK(rclc_executor_add_subscription(executor, &desired_rudder_angle_subscriber, &desired_rudder_angle_msg, &desired_rudder_angle_received_callback, ON_NEW_DATA));


    // -----------------------------------------------------
    // INITIALIZE THE CURRENT SAIL PUBLISHERS AND SUBSCRIBERS (ONLY FOR LUMPY)
    // -----------------------------------------------------
    #if BOAT_MODE == Lumpy
    RCCHECK(rclc_publisher_init_default(&current_sail_angle_publisher, &microros_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/current_sail_angle"));
    RCCHECK(rclc_publisher_init_default(&current_winch_angle_publisher,&microros_node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),"/current_winch_angle"));
    RCCHECK(rclc_subscription_init_best_effort(&desired_winch_angle_subscriber, &microros_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/actions/winch_angle"));
    RCCHECK(rclc_executor_add_subscription(executor, &desired_winch_angle_subscriber, &desired_winch_angle_msg, &desired_winch_angle_received_callback, ON_NEW_DATA));
    #endif


    // -----------------------------------------------------
    // INITIALIZE THE CURRENT HEADING PUBLISHER
    // -----------------------------------------------------
    RCCHECK(rclc_publisher_init_default(&compass_angle_publisher, &microros_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/heading"));


    // -----------------------------------------------------
    // INITIALIZE APPLICATION LOOP TIMER
    // -----------------------------------------------------    
    RCCHECK(rclc_timer_init_default(&application_loop_timer, support, RCL_MS_TO_NS(5), application_loop));
    RCCHECK(rclc_executor_add_timer(executor, &application_loop_timer));


    // -----------------------------------------------------
    //  INITIALIZE SPI AND I2C BUSSES
    // -----------------------------------------------------
    spi_init(SPI_PORT, 500 * 1000);
    gpio_set_function(SCLK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MISO_PIN, GPIO_FUNC_SPI);

    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);


    // -----------------------------------------------------
    //  INITIALIZE ALL CONNECTED DEVICES (SUCH AS COMPASS, ENCODER, CONTACTOR_DRIVER, STEPPER MOTOR DRIVER)
    // -----------------------------------------------------
    // AMT22 single turn is the encoder that we are using: 
        // https://www.sameskydevices.com/product/resource/amt22.pdf?srsltid=AfmBOorSuw8pg_K1wENhOY_XnnAFwlibV_cHx2dFayNq-89GXSfDkWn_
    // drv8711 is the chip for the stepper motor driver we are using: 
        // https://www.pololu.com/product/3730
        // https://www.ti.com/lit/ds/symlink/drv8711.pdf
    // CMPS14 is the compass module we are using: 
        // https://www.robot-electronics.co.uk/files/cmps14.pdf
    // For our contactor driver, we use an H-Bridge (VNH7100BAS):
        // https://www.st.com/resource/en/datasheet/vnh7100bas.pdf
        // https://www.st.com/resource/en/application_note/an5940-contactor-driver-using-the-vnh7100bas-stmicroelectronics.pdf

    AMT22_init(&rudderEncoder, RUDDER_ENCODER_CS_PIN, SPI_PORT);
    drv8711_init(&rudderStepperMotorDriver, SPI_PORT, RUDDER_MOTOR_CS_PIN, RUDDER_MOTOR_SLEEP_PIN, AutoMixed, MicroStep32, MAX_RUDDER_CURRENT);

    #if BOAT_MODE == Lumpy
    AMT22_init(&winchEncoder, WINCH_ENCODER_CS_PIN, SPI_PORT);
    drv8711_init(&winchStepperMotorDriver, SPI_PORT, WINCH_MOTOR_CS_PIN, WINCH_MOTOR_SLEEP_PIN, AutoMixed, MicroStep4, MAX_WINCH_CURRENT);
    #endif

    cmps14_init(&compass, I2C_PORT, 0x60);

    #if BOAT_MODE == Theseus
    initialize_contactor_driver(CONTACTOR_DRIVER_SEL0_PIN, CONTACTOR_DRIVER_PWM_PIN, CONTACTOR_DRIVER_IN_A_PIN, CONTACTOR_DRIVER_IN_B_PIN);
    pwm_motor_init();
    #endif


    // -----------------------------------------------------
    // INITIALIZE DEFAULT CALLBACK VARIABLE VALUES
    // -----------------------------------------------------
    desired_rudder_angle_msg.data = 0.0;
    current_rudder_angle_msg.data = 0.0;
    compass_angle_msg.data = 0.0;
}



// -----------------------------------------------------
// CALLBACK FUNCTIONS
// -----------------------------------------------------

void desired_rudder_angle_received_callback(const void *msg_in) {
    const std_msgs__msg__Float32 *desired_rudder_angle_msg = (const std_msgs__msg__Float32 *)msg_in;
    desired_rudder_angle = -(desired_rudder_angle_msg->data);

    if (desired_rudder_angle > MAX_RUDDER_ANGLE) 
        desired_rudder_angle = MAX_RUDDER_ANGLE;

    if (desired_rudder_angle < MIN_RUDDER_ANGLE)
        desired_rudder_angle = MIN_RUDDER_ANGLE;

    desired_rudder_motor_angle = get_motor_angle_from_rudder_angle(desired_rudder_angle);
}

void desired_winch_angle_received_callback(const void *msg_in) {
    const std_msgs__msg__Float32 *desired_sail_angle_msg = (const std_msgs__msg__Float32 *)msg_in;
    desired_sail_angle = desired_sail_angle_msg->data;

    if (desired_sail_angle > MAX_SAIL_ANGLE) 
        desired_sail_angle = MAX_SAIL_ANGLE;

    if (desired_sail_angle < MIN_SAIL_ANGLE)
        desired_sail_angle = MIN_SAIL_ANGLE;

    desired_winch_angle = get_winch_angle_from_sail_angle(desired_sail_angle);
}

void is_propeller_motor_enabled_callback(const void *msg_in) {
    
    const std_msgs__msg__Bool *is_propeller_motor_enabled_msg = (const std_msgs__msg__Bool *)msg_in;

    if (is_propeller_motor_enabled_msg->data)
        close_contactor(CONTACTOR_DRIVER_IN_A_PIN);
    else
        open_contactor(CONTACTOR_DRIVER_IN_A_PIN);
}

void zero_rudder_encoder_callback(const void *msg_in) {
    const std_msgs__msg__Bool *zero_msg = (const std_msgs__msg__Bool *)msg_in;
    
    if (zero_msg->data) {  // If message is true, zero the encoder
        zero_encoder_value(&rudderEncoder);
    }   
}

void zero_winch_encoder_callback(const void *msg_in) {
    const std_msgs__msg__Bool *zero_msg = (const std_msgs__msg__Bool *)msg_in;
    
    if (zero_msg->data) {  // If message is true, zero the encoder
        zero_encoder_value(&winchEncoder);
    }
}

void pwm_motor_callback(const void *msg_in) {
    const std_msgs__msg__Float32 *pwm_motor_msg = (const std_msgs__msg__Float32 *)msg_in;
    pwm_set_chan_level(0, PWM_CHAN_A, (uint16_t)((pwm_motor_msg->data / 100.0f) * 65535));
}

void pwm_motor_init() {
    gpio_set_function(0, GPIO_FUNC_PWM);

    uint slice_num = pwm_gpio_to_slice_num(0);

    pwm_set_wrap(slice_num, 3);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 1);
    pwm_set_enabled(slice_num, true);
}




// -----------------------------------------------------
// MAIN APPLICATION LOOP
// -----------------------------------------------------

void application_loop() {

    // -----------------------------------------------------
    // RUDDER CLOSED LOOP CONTROl
    // -----------------------------------------------------
    float current_rudder_motor_angle = get_motor_angle(&rudderEncoder) + RUDDER_ANGLE_OFFSET;     // motor_angle % 360
    if (current_rudder_motor_angle >= 180) {
        current_rudder_motor_angle -= 360;
    }

    float current_rudder_angle = get_rudder_angle_from_motor_angle(current_rudder_motor_angle);
    float rudder_error = current_rudder_angle - desired_rudder_angle;

    int number_of_steps = -10000;
    if (abs(rudder_error) > ACCEPTABLE_RUDDER_ERROR) {
        if (((int)rudder_error % 360) > 0 && ((int)rudder_error % 360) < 180) 
            drv8711_setDirection(&rudderStepperMotorDriver, COUNTER_CLOCKWISE);
    
        else 
            drv8711_setDirection(&rudderStepperMotorDriver, CLOCKWISE);

        number_of_steps = RUDDER_GAIN * abs(rudder_error) + RUDDER_GAIN_Q * pow(abs(rudder_error), 2);

        if (number_of_steps > 50) {
            number_of_steps = 50;
        }


        for (int i = 0; i < number_of_steps; i++) {
            drv8711_step(&rudderStepperMotorDriver);
            sleep_us(MIN_TIME_BETWEEN_MOTOR_STEPS_MICROSECONDS);
        }
    }

    compass_angle_msg.data = cmps14_getBearing(&compass) / 10.0;
    // compass_angle_msg.data = number_of_steps;
    current_rudder_angle_msg.data = current_rudder_angle;
    current_rudder_motor_angle_msg.data = current_rudder_motor_angle;

    rcl_publish(&current_rudder_motor_angle_publisher, &current_rudder_motor_angle_msg, NULL);
    rcl_publish(&current_rudder_angle_publisher, &current_rudder_angle_msg, NULL);
    rcl_publish(&compass_angle_publisher, &compass_angle_msg, NULL);


    // -----------------------------------------------------
    // SAIL CLOSED LOOP CONTROl
    // -----------------------------------------------------
    #if BOAT_MODE == Lumpy
    float current_winch_angle = get_motor_angle(&winchEncoder) + WINCH_ANGLE_OFFSET + 360 * get_turn_count(&winchEncoder);
    float current_sail_angle = get_sail_angle_from_winch_angle(current_winch_angle);


    // check for absurd errors, and if there is an absurd error, do nothing
    if (abs(sail_error) > 1000000 || sail_error != sail_error) 
        sail_error = 0;

    if (abs(sail_error) > ACCEPTABLE_SAIL_ERROR) {
        if (sail_error > 0) {
            drv8711_setDirection(&winchStepperMotorDriver, COUNTER_CLOCKWISE);
        }
        else {
            drv8711_setDirection(&winchStepperMotorDriver, CLOCKWISE);
        }

    // number of steps is some linear function that maps the error of the rudder to a number of steps we want to take per loop.
    // This ends up cooresponding to the speed of the rudder. The higher the rudder_error, the higher the speed of the rudder will be
        int number_of_steps = (int)(abs(sail_error) * WINCH_GAIN / MAX_SAIL_ERROR);  

        if (number_of_steps > 150) {
            number_of_steps = 150;
        }

        for (int i = 0; i < number_of_steps; i++) {
            drv8711_step(&winchStepperMotorDriver);
            sleep_us(MIN_TIME_BETWEEN_MOTOR_STEPS_MICROSECONDS);
        }
    }
    
    current_sail_angle_msg.data = current_sail_angle;
    current_winch_angle_msg.cleadata = current_winch_angle;

    rcl_publish(&current_winch_angle_publisher, &current_winch_angle_msg, NULL);
    rcl_publish(&current_sail_angle_publisher, &current_sail_angle_msg, NULL);
    #endif
}