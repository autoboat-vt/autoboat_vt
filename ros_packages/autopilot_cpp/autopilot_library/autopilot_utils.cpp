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
    float cosine_theta = std::clamp(dot / (magnitude1 * magnitude2), -1.0f, 1.0f);
    
    return std::acos(cosine_theta) * (180.0f / static_cast<float>(M_PI));
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
    // We generally want the angle to be counter clockwise from true east and
    // azimuths are generally given as clockwise from true north
    float bearing = std::fmod(-azimuth + 90.0f, 360.0f);
    if (bearing < 0) bearing += 360.0f;
    return bearing;
}

float get_distance_between_positions(const Position& position1, const Position& position2) {
    return get_distance(position1.latitude, position1.longitude, position2.latitude, position2.longitude);
}

bool is_angle_between_boundaries(float angle, float boundary1, float boundary2) {
    float angle_radians = angle * static_cast<float>(M_PI) / 180.0f;
    float boundary1_radians = boundary1 * static_cast<float>(M_PI) / 180.0f;
    float boundary2_radians = boundary2 * static_cast<float>(M_PI) / 180.0f;

    std::array<float, 2> angle_vector = {std::cos(angle_radians), std::sin(angle_radians)};
    std::array<float, 2> boundary1_vector = {std::cos(boundary1_radians), std::sin(boundary1_radians)};
    std::array<float, 2> boundary2_vector = {std::cos(boundary2_radians), std::sin(boundary2_radians)};

    return check_float_equivalence(
        get_angle_between_vectors(boundary1_vector, angle_vector) + get_angle_between_vectors(angle_vector, boundary2_vector),
        get_angle_between_vectors(boundary1_vector, boundary2_vector)
    );
}

bool does_line_violate_no_sail_zone(
    const std::array<float, 2>& current_waypoint,
    const std::array<float, 2>& destination_waypoint,
    float global_true_wind_angle,
    float no_sail_zone_size
) {
    float dx = destination_waypoint[0] - current_waypoint[0];
    float dy = destination_waypoint[1] - current_waypoint[1];
    float distance = std::sqrt(dx*dx + dy*dy);
    
    if (distance < 1e-6f) return false;

    // Upwind angle is opposite to wind
    float upwind_angle = std::fmod(global_true_wind_angle + 180.0f, 360.0f);
    float upwind_angle_radians = upwind_angle * static_cast<float>(M_PI) / 180.0f;
    
    std::array<float, 2> upwind_vector = {std::cos(upwind_angle_radians), std::sin(upwind_angle_radians)};
    std::array<float, 2> displacement_direction = {dx / distance, dy / distance};

    float angle_between = get_angle_between_vectors(upwind_vector, displacement_direction);
    
    return (angle_between < no_sail_zone_size);
}

bool does_line_segment_intersect_circle(
    const std::array<float, 2>& line_start,
    const std::array<float, 2>& line_end,
    const std::array<float, 2>& circle_position,
    float circle_radius
) {
    float dx = line_end[0] - line_start[0];
    float dy = line_end[1] - line_start[1];
    
    float fx = line_end[0] - circle_position[0];
    float fy = line_end[1] - circle_position[1];

    float a = dx*dx + dy*dy;
    float b = 2 * (fx*dx + fy*dy);
    float c = (fx*fx + fy*fy) - circle_radius*circle_radius;

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


