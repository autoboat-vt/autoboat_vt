#ifndef COMMON_LIBRARIES_H
#define COMMON_LIBRARIES_H

// Microros SDK Librareis
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>
#include <std_srvs/srv/empty.h>
#include <rmw_microros/rmw_microros.h>
#include <rcl_interfaces/msg/log.h>

// Pico SDK libraries
#include <pico/stdlib.h>

extern "C" {
#include <pico_uart_transports.h>
}

// Configuration files
#include "config.h"

// Misc libraries
#include <math.h>


// Checks whether the ros client (rclc) functions throw an error and if they do then turn on the light on the pico
#define RCCHECK(fn) {                   \
    rcl_ret_t temp_rc = fn;             \
    if ((temp_rc != RCL_RET_OK))        \
    {                                   \
        gpio_put(LED_PIN, 1);           \
    }                                   \
}


#endif