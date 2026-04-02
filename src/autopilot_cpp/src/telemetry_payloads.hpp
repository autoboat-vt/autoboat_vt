#ifndef TELEMETRY_PAYLOADS_HPP
#define TELEMETRY_PAYLOADS_HPP

#include <cstdint>
#include <string>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

#pragma pack(push, 1)

struct BoatStatusPayloadBase {
    float latitude = 0;
    float longitude = 0;
    float distance_to_next_waypoint = 0;
    float speed = 0;
    float velocity_x = 0;
    float velocity_y = 0;
    float desired_heading = 0;
    float heading = 0;
    float desired_rudder_angle = 0;
    float current_rudder_angle = 0;
    float rudder_angle_error = 0;
    uint8_t current_waypoint_index = 0;
    uint8_t autopilot_mode = 0;
};

struct SailboatStatusPayload : public BoatStatusPayloadBase {
    float true_wind_speed = 0;
    float true_wind_angle = 0;
    float apparent_wind_speed = 0;
    float apparent_wind_angle = 0;
    float current_sail_angle = 0;
    float desired_sail_angle = 0;
    float sail_angle_error = 0;
    uint8_t full_autonomy_maneuver = 0;
};

struct MotorboatStatusPayload : public BoatStatusPayloadBase {
    float rpm = 0;
    float duty_cycle = 0;
    float amp_hours = 0;
    float amp_hours_charged = 0;
    float current_to_vesc = 0;
    float voltage_to_motor = 0;
    float voltage_to_vesc = 0;
    float wattage_to_motor = 0;
    float motor_temperature = 0;
    float vesc_temperature = 0;
    float time_since_vesc_startup_in_ms = 0;
};

#pragma pack(pop)


uint8_t get_sailboat_mode(const std::string& mode);
uint8_t get_sailboat_state(const std::string& state);
uint8_t get_motorboat_mode(const std::string& mode);
json get_sailboat_mapping_raw();
json get_motorboat_mapping_raw();

#endif // TELEMETRY_PAYLOADS_HPP
