#ifndef CMPS14_I2CSENSOR_H  
#define CMPS14_I2CSENSOR_H 

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

#include "i2c_parent_class.h"

class cmps14 : public I2CDevice { 
    public:

    cmps14(i2c_inst_t* port, uint8_t address);

    int16_t getBearing();

    int8_t getPitch();

    int8_t getRoll();

    void readAccelerator(float *accelX, float *accelY, float *accelZ);

    void readGyro(float *gyroX, float *gyroY, float *gyroZ);

    void readMagnet(float *magnetX, float *magnetY, float *magnetZ);

    ~cmps14();



};//end cmps14

#endif