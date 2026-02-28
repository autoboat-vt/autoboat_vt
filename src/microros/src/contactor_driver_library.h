#ifndef CONTACTOR_DRIVER_LIBRARY_H
#define CONTACTOR_DRIVER_LIBRARY_H

#include "pico/stdlib.h"

void initialize_contactor_driver(int contactor_driver_in_a_pin) {

    gpio_init(contactor_driver_in_a_pin);
    gpio_set_dir(contactor_driver_in_a_pin, 1);
  
 }

void close_contactor(int contactor_driver_in_a_pin) {
    gpio_put(contactor_driver_in_a_pin, 0);
}

void open_contactor(int contactor_driver_in_a_pin) {
    gpio_put(contactor_driver_in_a_pin, 1);
}

#endif