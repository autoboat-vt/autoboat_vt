#include "vesc_protocol/vesc_protocol.hpp"
#include <cstring>
#include <iostream>

namespace vesc_cpp {

VescProtocol::VescProtocol() {
    nunchuck.valueX = 127;
    nunchuck.valueY = 127;
    nunchuck.lowerButton = false;
    nunchuck.upperButton = false;
}

std::vector<uint8_t> VescProtocol::packSendPayload(const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> message;
    
    if (payload.size() <= 256) {
        message.push_back(2);
        message.push_back(static_cast<uint8_t>(payload.size()));
    } else {
        message.push_back(3);
        message.push_back(static_cast<uint8_t>(payload.size() >> 8));
        message.push_back(static_cast<uint8_t>(payload.size() & 0xFF));
    }

    message.insert(message.end(), payload.begin(), payload.end());

    uint16_t crc = crc16(const_cast<uint8_t*>(payload.data()), payload.size());
    message.push_back(static_cast<uint8_t>(crc >> 8));
    message.push_back(static_cast<uint8_t>(crc & 0xFF));
    message.push_back(3);
    
    return message;
}

std::vector<uint8_t> VescProtocol::generateGetValues(uint8_t canId) {
    std::vector<uint8_t> payload;
    if (canId != 0) {
        payload.push_back(COMM_FORWARD_CAN);
        payload.push_back(canId);
    }
    payload.push_back(COMM_GET_VALUES);
    return packSendPayload(payload);
}

std::vector<uint8_t> VescProtocol::generateSetRPM(float rpm, uint8_t canId) {
    std::vector<uint8_t> payload;
    if (canId != 0) {
        payload.push_back(COMM_FORWARD_CAN);
        payload.push_back(canId);
    }
    payload.push_back(COMM_SET_RPM);
    
    int32_t index = 0;
    uint8_t buf[4];
    buffer_append_int32(buf, static_cast<int32_t>(rpm), &index);
    payload.insert(payload.end(), buf, buf + 4);
    
    return packSendPayload(payload);
}

std::vector<uint8_t> VescProtocol::generateSetDuty(float duty, uint8_t canId) {
    std::vector<uint8_t> payload;
    if (canId != 0) {
        payload.push_back(COMM_FORWARD_CAN);
        payload.push_back(canId);
    }
    payload.push_back(COMM_SET_DUTY);
    
    int32_t index = 0;
    uint8_t buf[4];
    buffer_append_int32(buf, static_cast<int32_t>(duty * 100000.0f), &index);
    payload.insert(payload.end(), buf, buf + 4);
    
    return packSendPayload(payload);
}

std::vector<uint8_t> VescProtocol::generateSetCurrent(float current, uint8_t canId) {
    std::vector<uint8_t> payload;
    if (canId != 0) {
        payload.push_back(COMM_FORWARD_CAN);
        payload.push_back(canId);
    }
    payload.push_back(COMM_SET_CURRENT);
    
    int32_t index = 0;
    uint8_t buf[4];
    buffer_append_int32(buf, static_cast<int32_t>(current * 1000.0f), &index);
    payload.insert(payload.end(), buf, buf + 4);
    
    return packSendPayload(payload);
}

std::vector<uint8_t> VescProtocol::generateSetBrakeCurrent(float brakeCurrent, uint8_t canId) {
    std::vector<uint8_t> payload;
    if (canId != 0) {
        payload.push_back(COMM_FORWARD_CAN);
        payload.push_back(canId);
    }
    payload.push_back(COMM_SET_CURRENT_BRAKE);
    
    int32_t index = 0;
    uint8_t buf[4];
    buffer_append_int32(buf, static_cast<int32_t>(brakeCurrent * 1000.0f), &index);
    payload.insert(payload.end(), buf, buf + 4);
    
    return packSendPayload(payload);
}

size_t VescProtocol::processReadPacket(const std::vector<uint8_t>& buffer) {
    if (buffer.size() < 2) return 0;
    
    size_t payload_len = 0;
    size_t header_len = 0;
    
    if (buffer[0] == 2) {
        payload_len = buffer[1];
        header_len = 2;
    } else if (buffer[0] == 3) {
        if (buffer.size() < 3) return 0;
        payload_len = (static_cast<size_t>(buffer[1]) << 8) | buffer[2];
        header_len = 3;
    } else {
        // Invalid start byte, caller should pop 1 byte and retry
        return 1;
    }
    
    size_t total_msg_len = header_len + payload_len + 3; // crc(2) + end_byte(1)
    
    if (buffer.size() < total_msg_len) {
        return 0; // Not enough data yet
    }
    
    if (buffer[total_msg_len - 1] != 3) {
        // End byte doesn't match, invalid packet, pop 1 and try to resync
        return 1;
    }
    
    // Verify CRC
    // We cannot construct a vector directly due to iterator constness mismatches
    std::vector<uint8_t> payload;
    for (size_t i = 0; i < payload_len; i++) {
        payload.push_back(buffer[header_len + i]);
    }

    uint16_t crc_calc = crc16(payload.data(), payload.size());
    uint16_t crc_msg = (static_cast<uint16_t>(buffer[total_msg_len - 3]) << 8) | buffer[total_msg_len - 2];
    
    if (crc_calc == crc_msg) {
        processPayload(payload.data(), payload.size());
        return total_msg_len;
    }
    
    // Invalid CRC
    return 1;
}

bool VescProtocol::processPayload(const uint8_t* payload, size_t length) {
    if (length == 0) return false;
    
    uint8_t packetId = payload[0];
    const uint8_t* message = payload + 1;
    int32_t index = 0;
    
    if (packetId == COMM_GET_VALUES) {
        data.tempMosfet = buffer_get_float16(message, 10.0, &index);
        data.tempMotor = buffer_get_float16(message, 10.0, &index);
        data.avgMotorCurrent = buffer_get_float32(message, 100.0, &index);
        data.avgInputCurrent = buffer_get_float32(message, 100.0, &index);
        index += 8; // Skip id and iq
        data.dutyCycleNow = buffer_get_float16(message, 1000.0, &index);
        data.rpm = buffer_get_float32(message, 1.0, &index);
        data.inpVoltage = buffer_get_float16(message, 10.0, &index);
        data.ampHours = buffer_get_float32(message, 10000.0, &index);
        data.ampHoursCharged = buffer_get_float32(message, 10000.0, &index);
        data.wattHours = buffer_get_float32(message, 10000.0, &index);
        data.wattHoursCharged = buffer_get_float32(message, 10000.0, &index);
        data.tachometer = buffer_get_int32(message, &index);
        data.tachometerAbs = buffer_get_int32(message, &index);
        data.error = static_cast<mc_fault_code>(message[index++]);
        data.pidPos = buffer_get_float32(message, 1000000.0, &index);
        data.id = message[index++];
        return true;
    }
    
    return false;
}

} // namespace vesc_cpp
