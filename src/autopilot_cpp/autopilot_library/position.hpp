
#pragma once

#include <tuple>
#include "geographic_function_library.hpp"


/*
A position that describes a point on the earth which is stored internally as its longitude and latitude.
Whenever a function such as get_local_coordinates is called, this class has to convert to the proper position measurement system such as NED or UTM.

This class mainly just calls on python libraries such as navpy, utm, and pygeodesy to convert to other position measurement systems.
*/
class Position {
    
    
public:


    Position(double longitude_, double latitude_) {
        longitude = longitude_;
        latitude = latitude_;
    }

    std::tuple<double, double> get_longitude_latitude() {
        return std::make_tuple(longitude, latitude);
    }

    std::tuple<double, double> get_latitude_longitude() {
        return std::make_tuple(latitude, longitude);
    }


    void set_longitude_latitude(double longitude_, double latitude_) {
        longitude = longitude_;
        latitude = latitude_;
    }


    // TODO: DOUBLE CHECK TO MAKE SURE THAT LATITUDE AND LONGITUDE ARE BEING RETURNED AND PROCESSED IN THE CORRECT ORDER
    // TODO: ALSO CLEAN THIS UP AND MAKE IT LOOK NICER
    // TODO: ALSO MAYBE CLEAN UP THE NED2LLA FUNCTION INTERFACE AND MAKE IT LOOK MORE LIKE THE LLA2NED FUNCTION INTERFACE
    void set_local_coordinates(double local_x, double local_y, double reference_longitude, double reference_latitude) {
        std::array<double, 3> lla_vector = ned2lla(std::array<double, 3>({local_y, local_x, 0}), reference_latitude, reference_longitude, 0);
        latitude = lla_vector[0];
        longitude = lla_vector[1];
    }


    std::array<float, 2> get_local_coordinates(double reference_longitude, double reference_latitude) {
        std::array<float, 3> ned_vector = lla2ned(latitude, longitude, 0, reference_latitude, reference_longitude, 0);
        return std::array<float, 2>({ned_vector[1], ned_vector[0]});
    }


    // def set_local_coordinates(self, local_x, local_y, reference_longitude, reference_latitude):
    //     self.latitude, self.longitude, _ = navpy.ned2lla([local_y, local_x, 0], reference_latitude, reference_longitude, 0)
        
    // def get_local_coordinates(self, reference_longitude_latitude: np.ndarray) -> np.ndarray[float]:
    //     reference_latitude = reference_longitude_latitude[1]
    //     reference_longitude = reference_longitude_latitude[0]
    //     local_y, local_x, _ = navpy.lla2ned(self.latitude, self.longitude, 0, reference_latitude, reference_longitude, 0)
    //     return np.array([local_x, local_y])


    double longitude, latitude;

};