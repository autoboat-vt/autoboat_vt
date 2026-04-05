#include "autopilot_utils.hpp"


std::string to_string(MotorboatAutopilotMode mode) {
    switch(mode) {
        case MotorboatAutopilotMode::Disabled: return "DISABLED";
        case MotorboatAutopilotMode::Full_RC: return "FULL_RC";
        case MotorboatAutopilotMode::Hold_Heading: return "HOLD_HEADING";
        case MotorboatAutopilotMode::Waypoint_Mission: return "WAYPOINT_MISSION";
    }
    return "DISABLED";
}

std::string to_string(SailboatAutopilotModes mode) {
    switch(mode) {
        case SailboatAutopilotModes::Disabled: return "DISABLED";
        case SailboatAutopilotModes::Full_RC: return "FULL_RC";
        case SailboatAutopilotModes::Hold_Best_Sail: return "HOLD_BEST_SAIL";
        case SailboatAutopilotModes::Hold_Heading: return "HOLD_HEADING";
        case SailboatAutopilotModes::Hold_Heading_And_Best_Sail: return "HOLD_HEADING_AND_BEST_SAIL";
        case SailboatAutopilotModes::Waypoint_Mission: return "WAYPOINT_MISSION";
    }
    return "DISABLED";
}

std::string to_string(SailboatStates state) {
    switch(state) {
        case SailboatStates::NORMAL: return "NORMAL";
        case SailboatStates::CW_TACKING: return "CW_TACKING";
        case SailboatStates::CCW_TACKING: return "CCW_TACKING";
        case SailboatStates::STALL: return "STALL";
    }
    return "NORMAL";
}


bool check_float_equivalence(float f1, float f2) {
    return std::abs(f1 - f2) <= 0.01f;
}

std::pair<float, float> cartesian_vector_to_polar(float x, float y) {
    if (x == 0.0f && y == 0.0f) return {0.0f, 0.0f};
    
    float magnitude = std::sqrt(x * x + y * y);
    float direction = std::atan2(y, x) * (180.0f / static_cast<float>(M_PI));
    
    // Normalize to [0, 360)
    direction = std::fmod(direction, 360.0f);
    if (direction < 0) direction += 360.0f;
    
    return {magnitude, direction};
}

float get_angle_between_vectors(const std::array<float, 2>& v1, const std::array<float, 2>& v2) {
    float magnitude1 = std::sqrt(v1[0]*v1[0] + v1[1]*v1[1]);
    float magnitude2 = std::sqrt(v2[0]*v2[0] + v2[1]*v2[1]);
    
    if (magnitude1 < 1e-6f || magnitude2 < 1e-6f) return 0.0f;

    float dot = v1[0]*v2[0] + v1[1]*v2[1];
    float cos_theta = std::clamp(dot / (magnitude1 * magnitude2), -1.0f, 1.0f);
    
    return std::acos(cos_theta) * (180.0f / static_cast<float>(M_PI));
}

float get_distance_between_angles(float angle1, float angle2) {
    float difference = std::fmod(angle1 - angle2 + 180.0f, 360.0f);
    if (difference < 0) difference += 360.0f;
    return -1.0f * (difference - 180.0f);
}

float get_bearing(const Position& current, const Position& dest) {
    // calculate_bearing returns clockwise from North (Azimuth)
    float azimuth = calculate_bearing(current.latitude, current.longitude, dest.latitude, dest.longitude);
    
    // Convert: North (0) -> 90, East (90) -> 0, etc.
    float bearing = std::fmod(-azimuth + 90.0f, 360.0f);
    if (bearing < 0) bearing += 360.0f;
    return bearing;
}

float get_distance_between_positions(const Position& position1, const Position& position2) {
    return get_distance(position1.latitude, position1.longitude, position2.latitude, position2.longitude);
}

bool is_angle_between_boundaries(float angle, float b1, float b2) {
    float a_rad = angle * static_cast<float>(M_PI) / 180.0f;
    float b1_rad = b1 * static_cast<float>(M_PI) / 180.0f;
    float b2_rad = b2 * static_cast<float>(M_PI) / 180.0f;

    std::array<float, 2> v_a = {std::cos(a_rad), std::sin(a_rad)};
    std::array<float, 2> v_b1 = {std::cos(b1_rad), std::sin(b1_rad)};
    std::array<float, 2> v_b2 = {std::cos(b2_rad), std::sin(b2_rad)};

    return check_float_equivalence(
        get_angle_between_vectors(v_b1, v_a) + get_angle_between_vectors(v_a, v_b2),
        get_angle_between_vectors(v_b1, v_b2)
    );
}

bool does_line_violate_no_sail_zone(
    const std::array<float, 2>& current,
    const std::array<float, 2>& dest,
    float global_true_wind_angle,
    float no_sail_zone_size
) {
    float dx = dest[0] - current[0];
    float dy = dest[1] - current[1];
    float dist = std::sqrt(dx*dx + dy*dy);
    
    if (dist < 1e-6f) return false;

    // Upwind angle is opposite to wind
    float upwind_angle = std::fmod(global_true_wind_angle + 180.0f, 360.0f);
    float upwind_rad = upwind_angle * static_cast<float>(M_PI) / 180.0f;
    
    std::array<float, 2> upwind_vector = {std::cos(upwind_rad), std::sin(upwind_rad)};
    std::array<float, 2> dir_vector = {dx / dist, dy / dist};

    float angle_diff = get_angle_between_vectors(upwind_vector, dir_vector);
    
    return (angle_diff < no_sail_zone_size);
}

bool does_line_segment_intersect_circle(
    const std::array<float, 2>& start,
    const std::array<float, 2>& end,
    const std::array<float, 2>& circle_pos,
    float radius
) {
    float dx = end[0] - start[0];
    float dy = end[1] - start[1];
    
    float fx = start[0] - circle_pos[0];
    float fy = start[1] - circle_pos[1];

    float a = dx*dx + dy*dy;
    float b = 2 * (fx*dx + fy*dy);
    float c = (fx*fx + fy*fy) - radius*radius;

    float discriminant = b*b - 4*a*c;
    
    if (discriminant < 0) return false;

    discriminant = std::sqrt(discriminant);
    float t1 = (-b - discriminant) / (2 * a);
    float t2 = (-b + discriminant) / (2 * a);

    // Check if the intersection points are within the line segment [0, 1]
    if (t1 >= 0 && t1 <= 1) return true;
    if (t2 >= 0 && t2 <= 1) return true;

    return false;
}


json yaml_to_json(const YAML::Node& node) {
    if (node.IsScalar()) {        
        bool b;
        if (YAML::convert<bool>::decode(node, b)) return b;
        
        int64_t i;
        if (YAML::convert<int64_t>::decode(node, i)) return i;
        
        float f;
        if (YAML::convert<float>::decode(node, f)) return f;
        
        return node.as<std::string>();
    }

    if (node.IsSequence()) {
        json j = json::array();
        for (const auto& item : node) {
            j.push_back(yaml_to_json(item));
        }
        return j;
    }
    
    if (node.IsMap()) {
        json j = json::object();
        for (auto it = node.begin(); it != node.end(); ++it) {
            j[it->first.as<std::string>()] = yaml_to_json(it->second);
        }
        return j;
    }

    return nullptr;
}
