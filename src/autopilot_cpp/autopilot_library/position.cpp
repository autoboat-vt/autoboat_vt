#include "position.hpp"
#include "geographic_function_library.hpp"


Position::Position(double longitude, double latitude) {
    this->longitude = longitude;
    this->latitude = latitude;
}


Position::Position(float local_x, float local_y, double reference_longitude, double reference_latitude) {
    set_local_coordinates(local_x, local_y, reference_longitude, reference_latitude);
}

void Position::set_longitude_latitude(double longitude, double latitude) {
    this->longitude = longitude;
    this->latitude = latitude;
}

std::array<double, 2> Position::get_longitude_latitude() const {
    return {longitude, latitude};
}

std::array<double, 2> Position::get_latitude_longitude() const {
    return {latitude, longitude};
}

void Position::set_local_coordinates(float local_x, float local_y, double reference_longitude, double reference_latitude) {
    // navpy style ned_vector is [North, East, Down]
    // local_x is North, local_y is East
    std::array<double, 3> lla_vector = ned2lla({local_x, local_y, 0.0}, reference_latitude, reference_longitude, 0.0);
    this->latitude = lla_vector[0];
    this->longitude = lla_vector[1];
}

std::array<float, 2> Position::get_local_coordinates(const std::array<double, 2>& reference_longitude_latitude) const {
    double reference_longitude = reference_longitude_latitude[0];
    double reference_latitude = reference_longitude_latitude[1];

    // lla2ned returns [North, East, Down]
    std::array<float, 3> ned_vector = lla2ned(this->latitude, this->longitude, 0.0, reference_latitude, reference_longitude, 0.0);
    
    // Return [North, East] which corresponds to [local_x, local_y]
    return {ned_vector[0], ned_vector[1]};
}
