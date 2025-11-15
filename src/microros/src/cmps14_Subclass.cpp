

#include "hardware/i2c.h"
#include <cmath>

//CMPS14 Register Definitions
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

#define ACCELEROMETER_SCALE 9.80592991914 / 1000.0 // 1 m/s^2
#define GYROSCOPE_SCALE 1.0 / 16.0              // 1 Dps
#define MAGNETOMETER_SCALE 1.0                   // No clue

#include "cmps14_Subclass.h"



cmps14::cmps14(i2c_inst_t* port, uint8_t address) : I2CDevice(port,address) {


}

int16_t cmps14::getBearing() {
    uint8_t buffer[2];
    if (readBytes(BEARING_Register, buffer, 2) != 2) { //0x02
        return -1;}

    int16_t result = ((buffer[0] << 8) | buffer[1]); // Glues 8 bits together to 16 bit, << shifts by 8 bits
    return result;
}

int8_t cmps14::getPitch() {
    uint8_t buffer; 
    if (readBytes(PITCH_Register, &buffer, 1) != 1) {
        return 0;
    }
    return (int8_t)buffer;
}

int8_t cmps14::getRoll() {
    uint8_t buffer;
    if (readBytes(ROLL_Register, &buffer, 1) != 1) {
        return 0;
    }
    return (int8_t)buffer;
}

void cmps14::readAccelerator(float *accelX, float *accelY, float *accelZ) {
    uint8_t buffer[6];
    if (readBytes(ACCELEROX_Register, buffer, 6) != 6) {
        *accelX = 0;
        *accelY = 0;
        *accelZ = 0;
        return;
    }
    *accelX = ((int16_t)(buffer[0] << 8 | buffer[1])) * ACCELEROMETER_SCALE;
    *accelY = ((int16_t)(buffer[2] << 8 | buffer[3])) * ACCELEROMETER_SCALE;
    *accelZ = ((int16_t)(buffer[4] << 8 | buffer[5])) * ACCELEROMETER_SCALE;
}
void cmps14::readGyro(float *gyroX, float *gyroY, float *gyroZ) {
    uint8_t buffer[6];
    if (readBytes(GYROX_Register, buffer, 6) != 6) {
        *gyroX = 0;
        *gyroY = 0;
        *gyroZ = 0;
        return;
    }
    *gyroX = ((int16_t)(buffer[0] << 8 | buffer[1])) * GYROSCOPE_SCALE;
    *gyroY = ((int16_t)(buffer[2] << 8 | buffer[3])) * GYROSCOPE_SCALE;
    *gyroZ = ((int16_t)(buffer[4] << 8 | buffer[5])) * GYROSCOPE_SCALE;
}

void cmps14::readMagnet(float *magnetX, float *magnetY, float *magnetZ) {
    uint8_t buffer[6];
    if (readBytes(MAGNETX_Register, buffer, 6) != 6) {
        *magnetX = 0;
        *magnetY = 0;
        *magnetZ = 0;
        return;
    }
    *magnetX = ((int16_t)(buffer[0] << 8 | buffer[1])) * MAGNETOMETER_SCALE;
    *magnetY = ((int16_t)(buffer[2] << 8 | buffer[3])) * MAGNETOMETER_SCALE;
    *magnetZ = ((int16_t)(buffer[4] << 8 | buffer[5])) * MAGNETOMETER_SCALE;
}

    cmps14::~cmps14(){
        //Cleanup Code, only primitives and no pointers[?], nothing needed
    }
