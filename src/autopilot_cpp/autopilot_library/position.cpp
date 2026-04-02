#include "position.hpp"
#include "geographic_function_library.hpp"


Position::Position(double longitude_, double latitude_) {
    longitude = longitude_;
    latitude = latitude_;
}

std::tuple<double, double> Position::get_longitude_latitude() {
    return std::make_tuple(longitude, latitude);
}

std::tuple<double, double> Position::get_latitude_longitude() {
    return std::make_tuple(latitude, longitude);
}

void Position::set_longitude_latitude(double longitude_, double latitude_) {
    longitude = longitude_;
    latitude = latitude_;
}

void Position::set_local_coordinates(double local_x, double local_y, double reference_longitude, double reference_latitude) {
    std::array<double, 3> lla_vector = ned2lla(std::array<double, 3>({local_y, local_x, 0}), reference_latitude, reference_longitude, 0);
    latitude = lla_vector[0];
    longitude = lla_vector[1];
}

std::array<float, 2> Position::get_local_coordinates(double reference_longitude, double reference_latitude) {
    std::array<float, 3> ned_vector = lla2ned(latitude, longitude, 0, reference_latitude, reference_longitude, 0);
    return std::array<float, 2>({ned_vector[1], ned_vector[0]});
}
