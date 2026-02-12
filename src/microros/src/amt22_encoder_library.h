#ifndef AMT22_ENCODER_LIBRARY_H
#define AMT22_ENCODER_LIBRARY_H

#include "hardware/spi.h"
#include "pico/stdlib.h"
#include <math.h>
#include <stdio.h>
#include <string.h>


#define READ_BIT 0x00

#define READ_RATE 10000

#define NO_OP 0x00
#define RESET_ENCODER 0x60
#define SET_ZERO_POINT 0x70
#define READ_TURNS 0xA0


typedef struct {
    spi_inst_t *spi_port;

    int PICO_SPI_CSN_PIN;
    float turn_count;
    float cur_angle;
} amt22;



inline void AMT22_init(amt22* encoder, int cs_pin, spi_inst_t *spi_port){
    encoder->spi_port = spi_port;

    encoder->PICO_SPI_CSN_PIN = cs_pin;

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(encoder->PICO_SPI_CSN_PIN);
    gpio_set_dir(encoder->PICO_SPI_CSN_PIN, GPIO_OUT);
    gpio_put(encoder->PICO_SPI_CSN_PIN, 1);
    gpio_pull_up(encoder->PICO_SPI_CSN_PIN);
    // sleep_us(10);
    // Make the CS pin available to picotool
    // bi_decl(bi_1pin_with_name(PICO_SPI_CSN_PIN, "SPI CS"));
}


inline int get_turn_count(amt22* encoder){
    return encoder->turn_count;
    
}

static inline void cs_select(amt22* encoder) {
    asm volatile("nop \n nop \n nop");
    asm volatile("nop \n nop \n nop");
    gpio_put(encoder->PICO_SPI_CSN_PIN, 0);  // Active low
    asm volatile("nop \n nop \n nop");
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect(amt22* encoder) {
    asm volatile("nop \n nop \n nop");
    asm volatile("nop \n nop \n nop");
    gpio_put(encoder->PICO_SPI_CSN_PIN, 1);
    asm volatile("nop \n nop \n nop");
    asm volatile("nop \n nop \n nop");
}


inline void zero_encoder_value(amt22* encoder){
    sleep_us(40);
    cs_select(encoder);
    sleep_us(3);
    uint8_t send[2] = {NO_OP, SET_ZERO_POINT};  
    spi_write_blocking(encoder->spi_port, send, 2);
    sleep_us(3);  
    cs_deselect(encoder);
}

static inline uint8_t* read_position(amt22* encoder, uint8_t * bytes_read){
    sleep_us(40);
    cs_select(encoder);
    sleep_us(3);
    uint8_t send[2] = {NO_OP, NO_OP};    
    spi_write_read_blocking(encoder->spi_port, send, bytes_read, 2);
    sleep_us(3);

    cs_deselect(encoder);
    
    return bytes_read;

}


static inline bool get_bit(uint8_t byte, int index){
    return (byte & 1 << (index)) != 0;
}

inline bool verify_packet(uint8_t packet_contents[2]){
    uint8_t first_byte = packet_contents[0];
    uint8_t second_byte = packet_contents[1];

    bool odd_parity = get_bit(first_byte, 7);
    bool even_parity = get_bit(first_byte, 6);

    bool odd_bits[7] = {
        get_bit(first_byte, 1), 
        get_bit(first_byte, 3),
        get_bit(first_byte, 5),
        get_bit(second_byte, 1),
        get_bit(second_byte, 3),
        get_bit(second_byte, 5),
        get_bit(second_byte, 7)
    };

    bool even_bits[7] = {
        get_bit(first_byte, 0), 
        get_bit(first_byte, 2),
        get_bit(first_byte, 4),
        get_bit(second_byte, 0),
        get_bit(second_byte, 2),
        get_bit(second_byte, 4),
        get_bit(second_byte, 6)
    };

     if (odd_parity == (odd_bits[0] ^ odd_bits[1] ^ odd_bits[2] ^ odd_bits[3] ^ odd_bits[4] ^ odd_bits[5] ^ odd_bits[6])) {
        return false;
    }
    if (even_parity == (even_bits[0] ^ even_bits[1] ^ even_bits[2] ^ even_bits[3] ^ even_bits[4] ^ even_bits[5] ^ even_bits[6])) {
        return false;
    }
    return true;
}

static inline float parse_angle(uint8_t packet_contents[2]){

    packet_contents[0] = packet_contents[0] & ~0b11000000;
    uint16_t angle_raw =packet_contents[0];
    angle_raw <<= 8;
    angle_raw |= packet_contents[1];

    angle_raw >>= 2; 
    float angle = ((float)angle_raw * 360) / (float)(pow(2, 12));

    return angle;
}

inline float get_motor_angle(amt22* encoder){
    sleep_us(READ_RATE);

    uint8_t* packet_array = (uint8_t*) malloc(2*sizeof(uint8_t));
    
    read_position(encoder, packet_array) ;
    if (verify_packet(packet_array) != 1) return encoder->cur_angle; // get_motor_angle(encoder);
    float next_angle = parse_angle(packet_array);
    free(packet_array);
    if (encoder->cur_angle > 270 && next_angle <90) {encoder->turn_count += 1;}
    if (encoder->cur_angle < 90 && next_angle > 270) {encoder->turn_count -= 1;}

    encoder->cur_angle = next_angle;
    return encoder->cur_angle;
}


#endif