#pragma once

#include <vector>
#include <cstdint>
#include <cstddef>
#include "vesc_protocol/datatypes.hpp"
#include "vesc_protocol/buffer.hpp"
#include "vesc_protocol/crc.hpp"

namespace vesc_cpp {

struct TelemetryData {
    float avgMotorCurrent;
    float avgInputCurrent;
    float dutyCycleNow;
    float rpm;
    float inpVoltage;
    float ampHours;
    float ampHoursCharged;
    float wattHours;
    float wattHoursCharged;
    long tachometer;
    long tachometerAbs;
    float tempMosfet;
    float tempMotor;
    float pidPos;
    uint8_t id;
    mc_fault_code error; 
};

struct NunchuckData {
    int valueX;
    int valueY;
    bool upperButton;
    bool lowerButton;
};

class VescProtocol {
public:
    VescProtocol();

    // Data containers updated by processing responses
    TelemetryData data;
    NunchuckData nunchuck;

    // Commands to generate payload bytes to be sent to VESC
    std::vector<uint8_t> generateGetValues(uint8_t canId = 0);
    std::vector<uint8_t> generateSetRPM(float rpm, uint8_t canId = 0);
    std::vector<uint8_t> generateSetDuty(float duty, uint8_t canId = 0);
    std::vector<uint8_t> generateSetCurrent(float current, uint8_t canId = 0);
    std::vector<uint8_t> generateSetBrakeCurrent(float brakeCurrent, uint8_t canId = 0);

    // Reads an incoming multi-byte buffer, identifies a valid VESC message frame,
    // verifies the CRC, updates struct data if COMM_GET_VALUES is returned,
    // and returns the number of consumed bytes from the buffer (so the caller can erase them).
    // Returns 0 if no full message could be extracted yet.
    size_t processReadPacket(const std::vector<uint8_t>& buffer);

private:
    std::vector<uint8_t> packSendPayload(const std::vector<uint8_t>& payload);
    bool processPayload(const uint8_t* payload, size_t length);
};

} // namespace vesc_cpp
