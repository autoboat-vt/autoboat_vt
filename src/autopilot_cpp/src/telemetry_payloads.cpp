#include "telemetry_payloads.hpp"


uint8_t get_sailboat_mode(const std::string& mode) {
    if (mode == "FULL_RC") return 1;
    if (mode == "HOLD_BEST_SAIL") return 2;
    if (mode == "HOLD_HEADING") return 3;
    if (mode == "HOLD_HEADING_AND_BEST_SAIL") return 4;
    if (mode == "WAYPOINT_MISSION") return 5;
    return 0; // DISABLED
}

uint8_t get_sailboat_state(const std::string& state) {
    if (state == "NORMAL") return 0;
    if (state == "CW_TACKING") return 1;
    if (state == "CCW_TACKING") return 2;
    if (state == "STALL") return 3;
    return 255; // NA
}

uint8_t get_motorboat_mode(const std::string& mode) {
    if (mode == "FULL_RC") return 1;
    if (mode == "HOLD_HEADING") return 2;
    if (mode == "WAYPOINT_MISSION") return 3;
    return 0; // DISABLED
}

json get_sailboat_mapping_raw() {
    return json::array({
        json::array({"latitude", "c_float"}),
        json::array({"longitude", "c_float"}),
        json::array({"distance_to_next_waypoint", "c_float"}),
        json::array({"speed", "c_float"}),
        json::array({"velocity_x", "c_float"}),
        json::array({"velocity_y", "c_float"}),
        json::array({"desired_heading", "c_float"}),
        json::array({"heading", "c_float"}),
        json::array({"desired_rudder_angle", "c_float"}),
        json::array({"current_rudder_angle", "c_float"}),
        json::array({"rudder_angle_error", "c_float"}),
        json::array({"current_waypoint_index", "c_ubyte"}),
        json::array({"autopilot_mode", "c_ubyte"}),
        json::array({"true_wind_speed", "c_float"}),
        json::array({"true_wind_angle", "c_float"}),
        json::array({"apparent_wind_speed", "c_float"}),
        json::array({"apparent_wind_angle", "c_float"}),
        json::array({"current_sail_angle", "c_float"}),
        json::array({"desired_sail_angle", "c_float"}),
        json::array({"sail_angle_error", "c_float"}),
        json::array({"full_autonomy_maneuver", "c_ubyte"})
    });
}

json get_motorboat_mapping_raw() {
    return json::array({
        json::array({"latitude", "c_float"}),
        json::array({"longitude", "c_float"}),
        json::array({"distance_to_next_waypoint", "c_float"}),
        json::array({"speed", "c_float"}),
        json::array({"velocity_x", "c_float"}),
        json::array({"velocity_y", "c_float"}),
        json::array({"desired_heading", "c_float"}),
        json::array({"heading", "c_float"}),
        json::array({"desired_rudder_angle", "c_float"}),
        json::array({"current_rudder_angle", "c_float"}),
        json::array({"rudder_angle_error", "c_float"}),
        json::array({"current_waypoint_index", "c_ubyte"}),
        json::array({"autopilot_mode", "c_ubyte"}),
        json::array({"rpm", "c_float"}),
        json::array({"duty_cycle", "c_float"}),
        json::array({"amp_hours", "c_float"}),
        json::array({"amp_hours_charged", "c_float"}),
        json::array({"current_to_vesc", "c_float"}),
        json::array({"voltage_to_motor", "c_float"}),
        json::array({"voltage_to_vesc", "c_float"}),
        json::array({"wattage_to_motor", "c_float"}),
        json::array({"motor_temperature", "c_float"}),
        json::array({"vesc_temperature", "c_float"}),
        json::array({"time_since_vesc_startup", "c_float"})
    });
}
