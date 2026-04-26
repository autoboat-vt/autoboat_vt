#ifndef CMPS14_COMPASS_LIBRARY_H
#define CMPS14_COMPASS_LIBRARY_H

#include "hardware/i2c.h"
#include <math.h>

// CMPS14 register definitions
#define CONTROL_Register 0x00
#define BEARING_Register 0x02
#define PITCH_Register 0x04
#define ROLL_Register 0x05
#define MAGNETX_Register 0x06
#define MAGNETY_Register 0x08
#define MAGNETZ_Register 0x0A
#define ACCELEROX_Register 0x20
#define ACCELEROY_Register 0x22
#define ACCELEROZ_Register 0x24
#define GYROX_Register 0x25
#define GYROY_Register 0x27
#define GYROZ_Register 0x29
#define ROLLP_Register 0x1C
#define PITCHP_Register 0x1A
#define Calibration_Register 0x1E



// Variables
typedef struct {
    int bearing;
    signed char pitch;
    signed char roll;
    uint8_t i2cAddress;
    i2c_inst_t i2cPort;
} cmps14;


#define ACCELEROMETER_SCALE 9.80592991914 / 1000.0 // 1 m/s^2
#define GYROSCOPE_SCALE 1.0 / 16.0              // 1 Dps
#define MAGNETOMETER_SCALE 1.0                   // No clue


void cmps14_init(cmps14* magnetometer, i2c_inst_t* port, uint8_t address) {
    magnetometer->bearing = 0;
    magnetometer->pitch = 0;
    magnetometer->roll = 0;
    magnetometer->i2cAddress = address;
    magnetometer->i2cPort = *port;
}

int cmps14_writeByte(cmps14* magnetometer, uint8_t reg, uint8_t value) {
    uint8_t data[] = {reg, value};
    int result = i2c_write_blocking(&(magnetometer->i2cPort), magnetometer->i2cAddress, data, 2, false);
    return result;
}

int cmps14_readBytes(cmps14* magnetometer, uint8_t reg, uint8_t *buffer, uint8_t length) {
    i2c_write_blocking(&(magnetometer->i2cPort), magnetometer->i2cAddress, &reg, 1, true);
    int result = i2c_read_blocking(&(magnetometer->i2cPort), magnetometer->i2cAddress, buffer, length, false);
    return result;
}

int16_t cmps14_getBearing(cmps14* magnetometer) {
    uint8_t buffer[2];
    if (cmps14_readBytes(magnetometer, BEARING_Register, buffer, 2) != 2) {
        return -1;
    }
    int16_t result = ((buffer[0] << 8) | buffer[1]);
    return result;
}

int8_t cmps14_getPitch(cmps14* magnetometer) {
    uint8_t buffer;
    if (cmps14_readBytes(magnetometer, PITCH_Register, &buffer, 1) != 1) {
        return 0;
    }
    return (int8_t)buffer;
}

int8_t cmps14_getRoll(cmps14* magnetometer) {
    uint8_t buffer;
    if (cmps14_readBytes(magnetometer, ROLL_Register, &buffer, 1) != 1) {
        return 0;
    }
    return (int8_t)buffer;
}

void cmps14_readAccelerator(cmps14* magnetometer, float *accelX, float *accelY, float *accelZ) {
    uint8_t buffer[6];
    if (cmps14_readBytes(magnetometer, ACCELEROX_Register, buffer, 6) != 6) {
        *accelX = 0;
        *accelY = 0;
        *accelZ = 0;
        return;
    }
    *accelX = ((int16_t)(buffer[0] << 8 | buffer[1])) * ACCELEROMETER_SCALE;
    *accelY = ((int16_t)(buffer[2] << 8 | buffer[3])) * ACCELEROMETER_SCALE;
    *accelZ = ((int16_t)(buffer[4] << 8 | buffer[5])) * ACCELEROMETER_SCALE;
}

void cmps14_readGyro(cmps14* magnetometer, float *gyroX, float *gyroY, float *gyroZ) {
    uint8_t buffer[6];
    if (cmps14_readBytes(magnetometer, GYROX_Register, buffer, 6) != 6) {
        *gyroX = 0;
        *gyroY = 0;
        *gyroZ = 0;
        return;
    }
    *gyroX = ((int16_t)(buffer[0] << 8 | buffer[1])) * GYROSCOPE_SCALE;
    *gyroY = ((int16_t)(buffer[2] << 8 | buffer[3])) * GYROSCOPE_SCALE;
    *gyroZ = ((int16_t)(buffer[4] << 8 | buffer[5])) * GYROSCOPE_SCALE;
}

void cmps14_readMagnet(cmps14* magnetometer, float *magnetX, float *magnetY, float *magnetZ) {
    uint8_t buffer[6];
    if (cmps14_readBytes(magnetometer, MAGNETX_Register, buffer, 6) != 6) {
        *magnetX = 0;
        *magnetY = 0;
        *magnetZ = 0;
        return;
    }
    *magnetX = ((int16_t)(buffer[0] << 8 | buffer[1])) * MAGNETOMETER_SCALE;
    *magnetY = ((int16_t)(buffer[2] << 8 | buffer[3])) * MAGNETOMETER_SCALE;
    *magnetZ = ((int16_t)(buffer[4] << 8 | buffer[5])) * MAGNETOMETER_SCALE;
}

#endif