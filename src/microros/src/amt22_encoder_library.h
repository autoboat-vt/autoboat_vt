#ifndef AMT22_ENCODER_LIBRARY_H
#define AMT22_ENCODER_LIBRARY_H

#include "hardware/spi.h"
#include "pico/stdlib.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "spi_device.h"

#define READ_BIT 0x00

#define READ_RATE 10000

#define NO_OP 0x00
#define RESET_ENCODER 0x60
#define SET_ZERO_POINT 0x70
#define READ_TURNS 0xA0

class amt22 : public spi_device
{

public:
    spi_inst_t *spi_port;
    int PICO_SPI_CSN_PIN;
    float turn_count;
    float cur_angle;

    amt22(int cs_pin, spi_inst_t *spi_port);

    int get_turn_count();

    void zero_encoder_value();

    static inline uint8_t *read_position(uint8_t *bytes_read);

    static inline bool get_bit(uint8_t byte, int index);

    bool verify_packet(uint8_t packet_contents[2]);

    static inline float parse_angle(uint8_t packet_contents[2]);

    float get_motor_angle();

private:
};

#endif
