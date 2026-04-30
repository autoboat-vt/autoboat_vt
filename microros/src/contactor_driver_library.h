#ifndef CONTACTOR_DRIVER_LIBRARY_H
#define CONTACTOR_DRIVER_LIBRARY_H

#include "pico/stdlib.h"

inline void initialize_contactor_driver(
        int contactor_driver_sel0_pin, 
        int contactor_driver_pwm_pin, 
        int contactor_driver_in_a_pin, 
        int contactor_driver_in_b_pin
    ) {

    gpio_init(contactor_driver_sel0_pin);
    gpio_set_dir(contactor_driver_sel0_pin, 1);
    gpio_pull_down(contactor_driver_sel0_pin);
    gpio_init(contactor_driver_pwm_pin);
    gpio_set_dir(contactor_driver_pwm_pin, 1);
    gpio_pull_down(contactor_driver_pwm_pin);
    gpio_init(contactor_driver_in_a_pin);
    gpio_set_dir(contactor_driver_in_a_pin, 1);
    gpio_init(contactor_driver_in_b_pin);
    gpio_set_dir(contactor_driver_in_b_pin, 1);

    gpio_put(contactor_driver_sel0_pin, 1);
    gpio_put(contactor_driver_pwm_pin, 1);
}

inline void close_contactor(int contactor_driver_in_a_pin) {
    gpio_put(contactor_driver_in_a_pin, 1);
}

inline void open_contactor(int contactor_driver_in_a_pin) {
    gpio_put(contactor_driver_in_a_pin, 0);
}

#endif