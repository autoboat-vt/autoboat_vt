#ifndef HAL_H
#define HAL_H

//TODO: include all driver libraries here
#include "spi_device.hpp"
#include "common_libraries.h"
#include "drv8711_stepper_motor_driver_library.hpp"
#include "i2c_device.hpp"
#ifdef __cplusplus
extern "C" {
#endif

#include <rcl/rcl.h>
#include <rclc/rclc.h>

#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/empty.h>
extern rmw_qos_profile_t best_effort_qos_profile; //{Pragya} why was this used

extern std_msgs__msg__Float64 desired_rudder_angle_msg;
extern std_msgs__msg__Float64 current_rudder_angle_msg;
extern std_msgs__msg__Float64 compass_angle_msg;




#ifdef __cplusplus
}
#endif


namespace HAL {
    //i think later i want to have initialize_lumpy_hal etc 
    //and then get rid of boat.cpp entirely besides the enum
    //remind myself to talk to elias about htis later

    //the other option is to move initialize_lumpy_hal and init_lumpy_microros
    //oput of microros and hal and into boat
    //This is recommended but i need to figure out what to do with microros.cpp

    //communication protocols
    void init_spi();
    void init_i2c();
    void init_uart();


    //device specifics
    void init_rudder_stepper(drv8711* rudderStepperMotorDriver);
    void init_contactor();
    void init_hydraulics();
    void init_magnetometer();
    void init_servo();
    void init_pump();
    void init_propeller();

    //debug
    u_int16_t debug(drv8711* rudderStepperMotorDriver);
}

#endif