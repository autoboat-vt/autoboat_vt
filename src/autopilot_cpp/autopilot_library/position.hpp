
#pragma once

#include <tuple>
#include <array>


/**
 * A position that describes a point on the earth which is stored internally as its longitude and latitude.
 * 
 * Whenever a function such as get_local_coordinates is called, this class has to convert to the proper
 * position measurement system such as NED or UTM.
 * 
 * This class mainly just calls on internal geographic functions to convert to other position measurement systems.
 * 
 * The constructor supports two mutually exclusive initialization schemes:
 * 1. Global: Provide longitude and latitude only.
 *    Position(longitude, latitude)
 * 2. Local: Provide local_x, local_y, and a reference global point (reference_longitude, reference_latitude).
 *    Position(local_x, local_y, reference_longitude, reference_latitude)
 */
class Position {
public:
    double longitude; ///< The global longitude in degrees.
    double latitude;  ///< The global latitude in degrees.

    /**
     * Global initialization: Sets the position using longitude and latitude.
     * @param longitude The global longitude in degrees.
     * @param latitude The global latitude in degrees.
     */
    Position(double longitude, double latitude);

    /**
     * Local initialization: Sets the position using local NED coordinates and a reference point.
     * @param local_x The Cartesian x-coordinate (NED North) relative to the reference point.
     * @param local_y The Cartesian y-coordinate (NED East) relative to the reference point.
     * @param reference_longitude The longitude of the origin for local coordinates.
     * @param reference_latitude The latitude of the origin for local coordinates.
     */
    Position(double local_x, double local_y, double reference_longitude, double reference_latitude);

    /**
     * Sets the position using longitude and latitude.
     * @param longitude The longitude to set the position to.
     * @param latitude The latitude to set the position to.
     */
    void set_longitude_latitude(double longitude, double latitude);

    /**
     * Gets the longitude and latitude of this position.
     * @return Array where the first element is longitude and the second is latitude.
     */
    std::array<double, 2> get_longitude_latitude() const;

    /**
     * Gets the latitude and longitude of this position.
     * @return Array where the first element is latitude and the second is longitude.
     */
    std::array<double, 2> get_latitude_longitude() const;

    /**
     * Sets the position using local NED coordinates.
     * @param local_x The local x coordinate (north).
     * @param local_y The local y coordinate (east).
     * @param reference_longitude The reference longitude for the NED frame.
     * @param reference_latitude The reference latitude for the NED frame.
     */
    void set_local_coordinates(double local_x, double local_y, double reference_longitude, double reference_latitude);

    /**
     * Gets the local NED coordinates of this position with respect to a reference position.
     * @param reference_longitude_latitude Array where [0] is reference longitude and [1] is reference latitude.
     * @return Array where [0] is local x (North) and [1] is local y (East).
     */
    std::array<double, 2> get_local_coordinates(const std::array<double, 2>& reference_longitude_latitude) const;
};