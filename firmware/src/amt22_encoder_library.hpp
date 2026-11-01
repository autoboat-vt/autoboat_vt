#ifndef AMT22_ENCODER_LIBRARY_H
#define AMT22_ENCODER_LIBRARY_H

#include "hardware/spi.h"
#include "pico/stdlib.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "spi_device.hpp"

#define READ_BIT 0x00

#define READ_RATE 10000

#define NO_OP 0x00
#define RESET_ENCODER 0x60
#define SET_ZERO_POINT 0x70
#define READ_TURNS 0xA0

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


// activate low

class amt22 : public spi_device
{

public:
    float turn_count;
    float cur_angle;

    amt22(int cs_pin, spi_inst_t *spi_port);

    uint8_t get_turn_count();

    void zero_encoder_value();

    uint8_t *read_position(uint8_t *bytes_read);

    bool get_bit(uint8_t byte, int index);

    bool verify_packet(uint8_t packet_contents[2]);

    float parse_angle(uint8_t packet_contents[2]);

    float get_motor_angle();

    void putLow();

private:
};

#endif