#ifndef CMPS14_I2CSENSOR_H  
#define CMPS14_I2CSENSOR_H 

#include "common_libraries.h"
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

#include "i2c_device.hpp"

using Vector3 = geometry_msgs__msg__Vector3;
using Quaternion = geometry_msgs__msg__Quaternion;

inline Vector3 operator+(const Vector3& lhs, const Vector3& rhs) {
    return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

// Rotate a vector into the orientation's frame. Orientation must be unit length.
inline Vector3 rotate(const Quaternion& orientation, const Vector3& vector) {
    const double tx = 2.0 * (orientation.y * vector.z - orientation.z * vector.y);
    const double ty = 2.0 * (orientation.z * vector.x - orientation.x * vector.z);
    const double tz = 2.0 * (orientation.x * vector.y - orientation.y * vector.x);

    return {
        vector.x + orientation.w * tx + orientation.y * tz - orientation.z * ty,
        vector.y + orientation.w * ty + orientation.z * tx - orientation.x * tz,
        vector.z + orientation.w * tz + orientation.x * ty - orientation.y * tx
    };
}

inline Quaternion toQuaternion(double roll, double pitch, double yaw) {
    // Inputs in radians
    double cr = cos(roll / 2);
    double sr = sin(roll / 2);
    double cp = cos(pitch / 2);
    double sp = sin(pitch / 2);
    double cy = cos(yaw / 2);
    double sy = sin(yaw / 2);

    return {
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
        cr*cp*cy + sr*sp*sy
    };
}

inline Vector3 calculateLinearAcceleration(
    const Vector3& accel,
    const Quaternion& orientation
) {
    Vector3 gravityWorld{0.0, 0.0, -9.80665};

    Vector3 accelWorld = rotate(orientation, accel);

    return accelWorld + gravityWorld;
}

inline Vector3 calculateAngularAcceleration(
    const Vector3& currentAngularVelocity,
    const Vector3& previousAngularVelocity,
    double dt
) {
    return {
        (currentAngularVelocity.x - previousAngularVelocity.x) / dt,
        (currentAngularVelocity.y - previousAngularVelocity.y) / dt,
        (currentAngularVelocity.z - previousAngularVelocity.z) / dt
    };
}

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
