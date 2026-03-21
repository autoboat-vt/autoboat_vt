#ifndef MAIN_MICROROS_NODE_H
#define MAIN_MICROROS_NODE_H

// Common microros and pico libraries
#include "common_libraries.h"

// Specific device libraries
#include "cmps14_compass_library.h"
#include "contactor_driver_library.h"
#include "amt22_encoder_library.h"
#include "drv8711_stepper_motor_driver_library.h"

#include "hardware/pwm.h"

#include <math.h>

#define CLOCKWISE 1
#define COUNTER_CLOCKWISE 0

#define MAX_RUDDER_CURRENT 2000
#define MAX_WINCH_CURRENT 2000

#if BOAT_MODE == Theseus
#define RUDDER_GAIN (float)2
#define RUDDER_GAIN_Q (float)0.5
#define RUDDER_NUMBER_OF_STEPS_TO_CLIP_AT 50
#define RUDDER_MICROSTEP MicroStep32
#else
#define RUDDER_GAIN (float)400
#define RUDDER_GAIN_Q (float)150
#define RUDDER_NUMBER_OF_STEPS_TO_CLIP_AT 50
#define RUDDER_MICROSTEP MicroStep4
#endif

#define WINCH_GAIN 150
#define WINCH_MICROSTEP MicroStep4
#define WINCH_NUMBER_OF_STEPS_TO_CLIP_AT 150

#define WINCH_ZERO_POINT 100

#define MAX_RUDDER_ANGLE 35
#define MIN_RUDDER_ANGLE -35

#define MAX_WINCH_ANGLE 580
#define MIN_WINCH_ANGLE -600

#define MAX_SAIL_ANGLE 90
#define MIN_SAIL_ANGLE 0

#define ACCEPTABLE_RUDDER_ERROR 0.1
#define ACCEPTABLE_WINCH_ERROR 0.5

#define RUDDER_ANGLE_OFFSET -85
#define WINCH_ANGLE_OFFSET 0
// #define COMPASS_OFFSET -13.7 // Magnetic Declination at WPI
#define COMPASS_OFFSET 180

const float MID_RUDDER_ANGLE = (MAX_RUDDER_ANGLE + MIN_RUDDER_ANGLE) / 2;
const float MID_WINCH_MOTOR_ANGLE = (MAX_WINCH_ANGLE + MIN_WINCH_ANGLE) / 2;

const int MAX_RUDDER_ERROR = (float)(MAX_RUDDER_ANGLE - MIN_RUDDER_ANGLE);
const int MAX_WINCH_ERROR = (float)(MAX_WINCH_ANGLE - MIN_RUDDER_ANGLE);
const int MAX_SAIL_ERROR = (float)(MAX_SAIL_ANGLE - MIN_SAIL_ANGLE);



// -----------------------------------------------------
// Polynomials
// -----------------------------------------------------

inline float get_rudder_angle_from_motor_angle(float motor_angle) {
    #if BOAT_MODE == Theseus
    return -0.00002094 * pow(motor_angle, 3) + 0.001259 * pow(motor_angle, 2) + 0.4159 * motor_angle - 8.373;
    #endif

    #if BOAT_MODE == Lumpy
    return motor_angle;
    #endif
}

inline float get_motor_angle_from_rudder_angle(float rudder_angle) {
    #if BOAT_MODE == Theseus
    return 0.001345 * pow(rudder_angle, 3) + 0.003741 * pow(rudder_angle, 2) + 2.142 * rudder_angle + 19.71;
    #endif

    #if BOAT_MODE == Lumpy
    return rudder_angle;
    #endif
}

inline float get_sail_angle_from_winch_angle(float winch_motor_angle) {
    return (winch_motor_angle - WINCH_ZERO_POINT) * 0.08087;
}

inline float get_winch_angle_from_sail_angle(float sail_angle) {
    return (sail_angle / 0.08087) + WINCH_ZERO_POINT;
}



// This period is the length of the delay between steps, which controls the
// stepper motor's speed.  You can increase the delay to make the stepper motor
// go slower.  If you decrease the delay, the stepper motor will go faster, but
// there is a limit to how fast it can go before it starts missing steps.
#define MIN_TIME_BETWEEN_MOTOR_STEPS_MICROSECONDS 2000 // microseconds


/**
 * @brief Initializes the microros node, publisher, and timer.
 * 
 * @param allocator The allocator to be used for memory allocation.
 * @param support   The ROS 2 support structure.
 * @param executor  The ROS 2 executor responsible for executing tasks.
*/

void application_init(rcl_allocator_t *allocator, rclc_support_t *support, rclc_executor_t *executor);

void uros_log(const char *msg, uint8_t severity, const char *file, const char *function);

void application_loop();

void desired_rudder_angle_received_callback(const void *msg_in);

void desired_sail_angle_received_callback(const void *msg_in);

void should_propeller_motor_be_powered_callback(const void *msg_in);

void zero_rudder_encoder_callback(const void *msg_in);

void zero_winch_encoder_callback(const void *msg_in);


#endif