#include "autopilot_utils.hpp"


std::string to_string(MotorboatControlModes mode) {
    switch(mode) {
        case MotorboatControlModes::DISABLED: return "DISABLED";
        case MotorboatControlModes::FULL_RC: return "FULL_RC";
        case MotorboatControlModes::HOLD_HEADING: return "HOLD_HEADING";
        case MotorboatControlModes::WAYPOINT_MISSION: return "WAYPOINT_MISSION";
        case MotorboatControlModes::EMERGENCY_STOP: return "EMERGENCY_STOP";
    }
    return "DISABLED";
}

std::string to_string(SailboatControlModes mode) {
    switch(mode) {
        case SailboatControlModes::DISABLED: return "DISABLED";
        case SailboatControlModes::FULL_RC: return "FULL_RC";
        case SailboatControlModes::HOLD_BEST_SAIL: return "HOLD_BEST_SAIL";
        case SailboatControlModes::HOLD_HEADING: return "HOLD_HEADING";
        case SailboatControlModes::HOLD_HEADING_AND_BEST_SAIL: return "HOLD_HEADING_AND_BEST_SAIL";
        case SailboatControlModes::WAYPOINT_MISSION: return "WAYPOINT_MISSION";
        case SailboatControlModes::EMERGENCY_STOP: return "EMERGENCY_STOP";
    }
    return "DISABLED";
}

std::string to_string(SailboatAutopilotStates state) {
    switch(state) {
        case SailboatAutopilotStates::NA: return "NA";
        case SailboatAutopilotStates::DOWNWIND_SAILING: return "DOWNWIND_SAILING";
        case SailboatAutopilotStates::PORT_TACK: return "PORT_TACK";
        case SailboatAutopilotStates::STARBOARD_TACK: return "STARBOARD_TACK";
        case SailboatAutopilotStates::CW_TACKING: return "CW_TACKING";
        case SailboatAutopilotStates::CCW_TACKING: return "CCW_TACKING";
        case SailboatAutopilotStates::STALL_WIGGLE_TO_PORT_TACK: return "STALL_WIGGLE_TO_PORT_TACK";
        case SailboatAutopilotStates::STALL_WIGGLE_TO_STARBOARD_TACK: return "STALL_WIGGLE_TO_STARBOARD_TACK";
    }
    return "NA";
}


bool check_float_equivalence(float f1, float f2) {
    return std::abs(f1 - f2) <= 0.001f;
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
    if (upwind_angle < 0.0f) upwind_angle += 360.0f;
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
    if (circle_radius <= 0.0f) {
        throw std::invalid_argument("Circle radius must be positive.");
    }

    float dx = line_end[0] - line_start[0];
    float dy = line_end[1] - line_start[1];

    float fx = line_start[0] - circle_position[0];
    float fy = line_start[1] - circle_position[1];

    float a = dx*dx + dy*dy;
    float ff = fx*fx + fy*fy;
    float r2 = circle_radius * circle_radius;

    if (ff <= r2) return true;
    if (a == 0.0f) return ff <= r2;

    float b = 2.0f * (fx*dx + fy*dy);
    float c = ff - r2;

    float discriminant = b*b - 4.0f*a*c;

    if (discriminant < 0.0f) return false;

    discriminant = std::sqrt(discriminant);
    float t1 = (-b - discriminant) / (2.0f * a);
    float t2 = (-b + discriminant) / (2.0f * a);

    return (t1 >= 0.0f && t1 <= 1.0f) || (t2 >= 0.0f && t2 <= 1.0f);
}

bool is_angle_between_boundaries_with_hysteresis(
    float angle, float boundary1, float boundary2,
    bool biased_side, float hysteresis_amount
) {
    float test_angle1 = std::fmod(angle + hysteresis_amount, 360.0f);
    if (test_angle1 < 0) test_angle1 += 360.0f;
    float test_angle2 = std::fmod(angle - hysteresis_amount, 360.0f);
    if (test_angle2 < 0) test_angle2 += 360.0f;

    bool is_test_angle1_between_boundaries = is_angle_between_boundaries(test_angle1, boundary1, boundary2);
    bool is_test_angle2_between_boundaries = is_angle_between_boundaries(test_angle2, boundary1, boundary2);

    if (!biased_side) {
        return is_test_angle1_between_boundaries && is_test_angle2_between_boundaries;
    } else {
        return is_test_angle1_between_boundaries || is_test_angle2_between_boundaries;
    }
}
