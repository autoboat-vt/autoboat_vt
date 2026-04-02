
#pragma once

#include <tuple>
#include <array>


/*
A position that describes a point on the earth which is stored internally as its longitude and latitude.
Whenever a function such as get_local_coordinates is called, this class has to convert to the proper position measurement system such as NED or UTM.

This class mainly just calls on python libraries such as navpy, utm, and pygeodesy to convert to other position measurement systems.
*/
class Position {
    
    
public:

    double longitude, latitude;

    Position(double longitude_, double latitude_);

    std::tuple<double, double> get_longitude_latitude();
    std::tuple<double, double> get_latitude_longitude();
    void set_longitude_latitude(double longitude_, double latitude_);

    // TODO: DOUBLE CHECK TO MAKE SURE THAT LATITUDE AND LONGITUDE ARE BEING RETURNED AND PROCESSED IN THE CORRECT ORDER
    // TODO: ALSO CLEAN THIS UP AND MAKE IT LOOK NICER
    // TODO: ALSO MAYBE CLEAN UP THE NED2LLA FUNCTION INTERFACE AND MAKE IT LOOK MORE LIKE THE LLA2NED FUNCTION INTERFACE
    void set_local_coordinates(double local_x, double local_y, double reference_longitude, double reference_latitude);
    std::array<float, 2> get_local_coordinates(double reference_longitude, double reference_latitude);
};