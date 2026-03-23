#include "systems.hpp"

Systems::Systems(boat_type bt){
    current_boat = bt;
    initalize_cores(); //assumption is application loop will be called

}

void Systems::initalize_cores(){
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_executor_init(&executor, &support.context, 5, &allocator);
}

void Systems::initialize_microros(){
        microros_node.set_cores(&allocator, &support, &executor);
        //change this to rely on boat system
        microros_node.initialize_theseus_peripherals();
}

void Systems::check_microros(){
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}

void Systems::initialize_hal(){
    //write boat specific intialization later 
    HAL::init_spi();
}

void Systems::cleanup(){
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);
}


void Systems::initialize_application_loop(){
    rclc_timer_init_default(&application_loop_timer, &support, RCL_MS_TO_NS(1), application_loop);
    rclc_executor_add_timer(&executor, &application_loop_timer);
}

void Systems::application_loop(rcl_timer_t * timer, int64_t last_call_time)
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