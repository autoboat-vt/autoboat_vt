//
// This is just a library of functions taken from geopy, navpy, and pyproj
// This implements basic distance, azimuth calculations as well as coordinate transformations 
// between NED (North, East, Down), LLA (Latitude, Longitude, Altitude), and ECEF (Earth-centered, Earth-fixed) coordinate systems
//


// Here are some good resources to look at if you want to learn more about this function library:
// https://www.codeguru.com/cplusplus/geographic-distance-and-azimuth-calculations/
// https://en.wikipedia.org/wiki/Haversine_formula
// https://en.wikipedia.org/wiki/Vincenty%27s_formulae 
// https://github.com/NavPy/NavPy/blob/master/navpy/core/navpy.py
// https://en.wikipedia.org/wiki/Earth-centered,_Earth-fixed_coordinate_system
// https://en.wikipedia.org/wiki/Geographic_coordinate_system
// https://en.wikipedia.org/wiki/World_Geodetic_System


#pragma once

#include <tuple>
#include <cmath>
#include <array>
#include <vector>
#include <stdexcept>


// choose vincenty for more precise but slower calculations
// choose haversine for less precise but faster calculations
constexpr std::string DISTANCE_FUNCTION_TO_USE = "haversine";


// Constants taken from here: https://en.wikipedia.org/wiki/World_Geodetic_System
constexpr double WGS84_A = 6378137.0;              // semi-major axis
constexpr double WGS84_E2 = 6.69437999014e-3;      // eccentricity^2
constexpr double WGS84_F = 1 / 298.257223563;     // flattening



double get_distance(double latitude1, double longitude1, double latitude2, double longitude2) {
    if (DISTANCE_FUNCTION_TO_USE == "vincenty") 
        return get_distance_vincenty(latitude1, longitude1, latitude2, longitude2);

    else if (DISTANCE_FUNCTION_TO_USE == "haversine")
        return get_distance_haversine(latitude1, longitude1, latitude2, longitude2);

    else 
        throw std::runtime_error("DISTANCE_FUNCTION_TO_USE must be either vincenty or haversine");
}



// haversine distance for distance between 2 points on a spheroid (approximation to an elipsoid)
// this is a less accurate distance function than vincenty since it assumes that the earth is a perfect sphere
// https://en.wikipedia.org/wiki/Haversine_formula
double get_distance_haversine(double latitude1, double longitude1, double latitude2, double longitude2) {
    // Haversine in meters
    const double R = WGS84_A;
    double dlat = (latitude2 - latitude1) * M_PI / 180.0;
    double dlon = (longitude2 - longitude1) * M_PI / 180.0;
    double a = sin(dlat/2)*sin(dlat/2) + cos(latitude1*M_PI/180.0)*cos(latitude2*M_PI/180.0)*sin(dlon/2)*sin(dlon/2);
    double c = 2*atan2(sqrt(a), sqrt(1-a));
    return R * c;
}



// Using vincentys formula to calculate the distance between 2 points on an elipsoid
// this is a more accurate distance function than haversine since it doesn't make the assumption that the earth is a perfect sphere
// https://github.com/dariusarnold/vincentys-formula
// https://en.wikipedia.org/wiki/Vincenty%27s_formulae 
double get_distance_vincenty(double latitude1, double longitude1, double latitude2, double longitude2) {
    using namespace std;
    constexpr double req = WGS84_A;             //Radius at equator
    constexpr double flat = WGS84_F;    //flattening of earth
    constexpr double rpol = (1 - flat) * req;

    double sin_sigma, cos_sigma, sigma, sin_alpha, cos_sq_alpha, cos2sigma;
    double C, lam_pre;

    // convert to radians
    latitude1 = M_PI * latitude1 / 180.0;
    latitude2 = M_PI * latitude2 / 180.0;
    longitude1 = M_PI * longitude1 / 180.0;
    longitude2 = M_PI * longitude2 / 180.0;

    const double u1 = atan((1 - flat) * tan(latitude2));
    const double u2 = atan((1 - flat) * tan(latitude1));

    double lon = longitude1 - longitude2;
    double lam = lon;
    constexpr double tol = 10.e-12; // iteration tolerance
    double diff = 1.;

    while (abs(diff) > tol) {
        sin_sigma = sqrt(pow((cos(u2) * sin(lam)), 2.) + pow(cos(u1)*sin(u2) - sin(u1)*cos(u2)*cos(lam), 2.));
        if (sin_sigma == 0.) {
            // Coincident points, prevent division by zero resulting in NaN.
            return {0., 0.};
        }
        cos_sigma = sin(u1) * sin(u2) + cos(u1) * cos(u2) * cos(lam);
        sigma = atan(sin_sigma / cos_sigma);
        if (sigma <= 0) sigma = M_PI + sigma;
        sin_alpha = (cos(u1) * cos(u2) * sin(lam)) / sin_sigma;
        cos_sq_alpha = 1 - pow(sin_alpha, 2.);
        if (cos_sq_alpha == 0.) {
            cos2sigma = 0.;
        } else {
            cos2sigma = cos_sigma - ((2 * sin(u1) * sin(u2)) / cos_sq_alpha);
        }
        C = (flat / 16) * cos_sq_alpha * (4 + flat * (4 - 3 * cos_sq_alpha));
        lam_pre = lam;
        lam = lon + (1 - C) * flat * sin_alpha * (sigma + C * sin_sigma * (cos2sigma + C * cos_sigma * (2 * pow(cos2sigma, 2.) - 1)));
        diff = abs(lam_pre - lam);
    }

    const double usq = cos_sq_alpha * ((pow(req, 2.) - pow(rpol, 2.)) / pow(rpol ,2.));
    const double A = 1 + (usq / 16384) * (4096 + usq * (-768 + usq * (320 - 175 * usq)));
    const double B = (usq / 1024) * (256 + usq * (-128 + usq * (74 - 47 * usq)));
    const double delta_sig = B * sin_sigma * (cos2sigma + 0.25 * B * (cos_sigma * (-1 + 2 * pow(cos2sigma, 2.)) - (1. / 6) * B * cos2sigma * (-3 + 4 * pow(sin_sigma, 2.)) * (-3 + 4 * pow(cos2sigma, 2.))));
    const double dis = rpol * A * (sigma - delta_sig);
    const double azi1 = atan2((cos(u2) * sin(lam)), (cos(u1) * sin(u2) - sin(u1) * cos(u2) * cos(lam)));

    return dis;
}




/**
 * Calculates the initial azimuth (bearing) from point 1 to point 2.
 * 
 * @param latitude1 Latitude of point 1 in degrees.
 * @param longitude1 Longitude of point 1 in degrees.
 * @param latitude2 Latitude of point 2 in degrees.
 * @param longitude2 Longitude of point 2 in degrees.
 * @return Azimuth in degrees, clockwise from North (0 to 360).
 */
double calculate_bearing(double latitude1, double longitude1, double latitude2, double longitude2) {
   
    // Convert degrees to radians
    latitude1 = (latitude1 * M_PI / 180.0);
    longitude1 = (longitude1 * M_PI / 180.0);
    latitude2 = (latitude2 * M_PI / 180.0);
    longitude2 = (longitude2 * M_PI / 180.0);

    const double deltaLon = longitude2 - longitude1;

    // Calculate the components for atan2
    // x = cos(φ1) * sin(φ2) - sin(φ1) * cos(φ2) * cos(Δλ)
    // y = sin(Δλ) * cos(φ2)
    // where φ is latitude and λ is longitude
    const double y = std::sin(deltaLon) * std::cos(latitude2);
    const double x = std::cos(latitude1) * std::sin(latitude2) - std::sin(latitude1) * std::cos(latitude2) * std::cos(deltaLon);

    // Use atan2(y, x) to get the angle in radians (range -PI to +PI)
    double azimuthRad = std::atan2(y, x);

    // Convert result to degrees
    double azimuthDeg = (azimuthRad * 180.0 / M_PI);

    // Normalize to 0-360 degrees (azimuth is typically measured clockwise from North)
    if (azimuthDeg < 0) {
        azimuthDeg += 360.0;
    }

    return azimuthDeg;
}






// LLA, ECEF, and NED transformation functions are adapted from: https://github.com/NavPy/NavPy/blob/master/navpy/core/navpy.py

// -------------------------
// LLA to ECEF
// -------------------------
inline std::array<double,3> lla2ecef(double lat_deg, double lon_deg, double alt_m)
{
    double lat = lat_deg * M_PI / 180.0;
    double lon = lon_deg * M_PI / 180.0;

    double sinLat = std::sin(lat);
    double cosLat = std::cos(lat);
    double cosLon = std::cos(lon);
    double sinLon = std::sin(lon);

    double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sinLat * sinLat);

    double x = (N + alt_m) * cosLat * cosLon;
    double y = (N + alt_m) * cosLat * sinLon;
    double z = (N * (1 - WGS84_E2) + alt_m) * sinLat;

    return {x, y, z};
}

// -------------------------
// ECEF to LLA (iterative)
// -------------------------
inline std::array<double,3> ecef2lla(double x, double y, double z)
{
    double lon = std::atan2(y, x);

    double p = std::sqrt(x*x + y*y);
    double lat = std::atan2(z, p * (1 - WGS84_E2));

    double h = 0;
    double lat_prev;

    // Iterate like Python code
    for(int i=0; i<100; i++)
    {
        lat_prev = lat;
        double sinLat = std::sin(lat);
        double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sinLat * sinLat);

        if (std::abs(M_PI/2 - std::abs(lat)) > 1e-3)
            h = p / std::cos(lat) - N;
        else
            h = z / sinLat - N * (1 - WGS84_E2);

        lat = std::atan2(z + WGS84_E2 * N * sinLat, p);

        if (std::abs(lat - lat_prev) < 1e-12)
            break;
    }

    return {
        lat * 180.0 / M_PI,
        lon * 180.0 / M_PI,
        h
    };
}

// Build ECEFtoNED rotation matrix
inline std::array<std::array<double,3>,3> nedRotation(double lat_deg, double lon_deg)
{
    double lat = lat_deg * M_PI / 180.0;
    double lon = lon_deg * M_PI / 180.0;

    double sinLat = std::sin(lat), cosLat = std::cos(lat);
    double sinLon = std::sin(lon), cosLon = std::cos(lon);

    return {{
        {-sinLat*cosLon, -sinLat*sinLon,  cosLat},
        {-sinLon,         cosLon,         0     },
        {-cosLat*cosLon, -cosLat*sinLon, -sinLat}
    }};
}





// multiply matrix * vector
inline std::array<double,3> matmul(const std::array<std::array<double,3>,3> &C, const std::array<double,3> &v) {
    return {
        C[0][0]*v[0] + C[0][1]*v[1] + C[0][2]*v[2],
        C[1][0]*v[0] + C[1][1]*v[1] + C[1][2]*v[2],
        C[2][0]*v[0] + C[2][1]*v[1] + C[2][2]*v[2]
    };
}



// -------------------------
// ECEF to NED
// -------------------------
inline std::array<double,3> ecef2ned(const std::array<double,3> &ecef_vector, double reference_latitude, double reference_longitude) {
    auto C = nedRotation(reference_latitude, reference_longitude);
    return matmul(C, ecef_vector);
}

// -------------------------
// NED to ECEF
// -------------------------
inline std::array<double,3> ned2ecef(const std::array<double,3> &ned_vector, double reference_latitude, double reference_longitude) {
    auto C = nedRotation(reference_latitude, reference_longitude);

    // Use transpose
    std::array<std::array<double,3>,3> Ct {{
        {C[0][0], C[1][0], C[2][0]},
        {C[0][1], C[1][1], C[2][1]},
        {C[0][2], C[1][2], C[2][2]}
    }};

    return matmul(Ct, ned_vector);
}

// -------------------------
// LLA to NED
// -------------------------
inline std::array<double,3> lla2ned(double latitude, double longitude, double altitude, double reference_latitude, double reference_longitude, double reference_altitude) {
    auto e = lla2ecef(latitude, longitude, altitude);
    auto e0 = lla2ecef(reference_latitude, reference_longitude, reference_altitude);

    return ecef2ned({e[0]-e0[0], e[1]-e0[1], e[2]-e0[2]}, reference_latitude, reference_longitude);
}

// -------------------------
// NED to LLA
// -------------------------
inline std::array<double,3> ned2lla(const std::array<double,3> &ned_vector, double reference_latitude, double reference_longitude, double reference_altitude) {
    auto e0 = lla2ecef(reference_latitude, reference_longitude, reference_altitude);
    auto e_rel = ned2ecef(ned_vector, reference_latitude, reference_longitude);

    std::array<double,3> ecef = {
        e0[0] + e_rel[0],
        e0[1] + e_rel[1],
        e0[2] + e_rel[2]
    };

    return ecef2lla(ecef[0], ecef[1], ecef[2]);
}







// CODE FROM PYPROJ IF WE EVER NEED IT

// def inv(
//     self,
//     lons1: Any,
//     lats1: Any,
//     lons2: Any,
//     lats2: Any,
//     radians: bool = False,
//     inplace: bool = False,
//     return_back_azimuth: bool = True,
// ) -> tuple[Any, Any, Any]:
//     """

//     Inverse transformation

//     Determine forward and back azimuths, plus distances
//     between initial points and terminus points.

//     .. versionadded:: 3.5.0 inplace
//     .. versionadded:: 3.5.0 return_back_azimuth

//     Accepted numeric scalar or array:

//     - :class:`int`
//     - :class:`float`
//     - :class:`numpy.floating`
//     - :class:`numpy.integer`
//     - :class:`list`
//     - :class:`tuple`
//     - :class:`array.array`
//     - :class:`numpy.ndarray`
//     - :class:`xarray.DataArray`
//     - :class:`pandas.Series`

//     Parameters
//     ----------
//     lons1: scalar or array
//         Longitude(s) of initial point(s)
//     lats1: scalar or array
//         Latitude(s) of initial point(s)
//     lons2: scalar or array
//         Longitude(s) of terminus point(s)
//     lats2: scalar or array
//         Latitude(s) of terminus point(s)
//     radians: bool, default=False
//         If True, the input data is assumed to be in radians.
//         Otherwise, the data is assumed to be in degrees.
//     inplace: bool, default=False
//         If True, will attempt to write the results to the input array
//         instead of returning a new array. This will fail if the input
//         is not an array in C order with the double data type.
//     return_back_azimuth: bool, default=True
//         If True, the second return value (azi21) will be the back azimuth
//         (flipped 180 degrees), Otherwise, it will also be a forward azimuth.

//     Returns
//     -------
//     scalar or array:
//         Forward azimuth(s) (azi12)
//     scalar or array:
//         Back azimuth(s) or Forward azimuth(s) (azi21)
//     scalar or array:
//         Distance(s) between initial and terminus point(s)
//         in meters
//     """
//     try:
//         # Fast-path for scalar input, will raise if invalid types are input
//         # and we can fallback below
//         return self._inv_point(
//             lons1,
//             lats1,
//             lons2,
//             lats2,
//             radians=radians,
//             return_back_azimuth=return_back_azimuth,
//         )
//     except TypeError:
//         pass

//     # process inputs, making copies that support buffer API.
//     inx, x_data_type = _copytobuffer(lons1, inplace=inplace)
//     iny, y_data_type = _copytobuffer(lats1, inplace=inplace)
//     inz, z_data_type = _copytobuffer(lons2, inplace=inplace)
//     ind = _copytobuffer(lats2, inplace=inplace)[0]
//     self._inv(
//         inx, iny, inz, ind, radians=radians, return_back_azimuth=return_back_azimuth
//     )
//     # if inputs were lists, tuples or floats, convert back.
//     outx = _convertback(x_data_type, inx)
//     outy = _convertback(y_data_type, iny)
//     outz = _convertback(z_data_type, inz)
//     return outx, outy, outz



// @cython.boundscheck(False)
// @cython.wraparound(False)
// def _inv(
//     self,
//     object lons1,
//     object lats1,
//     object lons2,
//     object lats2,
//     bint radians=False,
//     bint return_back_azimuth=True,
// ):
//     """
//     inverse transformation - return forward azimuth (azi12) and back azimuths (azi21), plus distance
//     between an initial and terminus lat/lon pair.
//     if radians=True, lons/lats are radians instead of degree
//     if return_back_azimuth=True, azi21 is a back azimuth (180 degrees flipped),
//     otherwise azi21 is also a forward azimuth.
//     """
//     cdef:
//         PyBuffWriteManager lon1buff = PyBuffWriteManager(lons1)
//         PyBuffWriteManager lat1buff = PyBuffWriteManager(lats1)
//         PyBuffWriteManager lon2buff = PyBuffWriteManager(lons2)
//         PyBuffWriteManager lat2buff = PyBuffWriteManager(lats2)

//     # process data in buffer
//     if not lon1buff.len == lat1buff.len == lon2buff.len == lat2buff.len:
//         raise GeodError("Array lengths are not the same.")

//     cdef:
//         double lat1
//         double lon1
//         double lat2
//         double lon2
//         double pazi1
//         double pazi2
//         double ps12
//         Py_ssize_t iii

//     with nogil:
//         for iii in range(lon1buff.len):
//             if radians:
//                 lon1 = _RAD2DG * lon1buff.data[iii]
//                 lat1 = _RAD2DG * lat1buff.data[iii]
//                 lon2 = _RAD2DG * lon2buff.data[iii]
//                 lat2 = _RAD2DG * lat2buff.data[iii]
//             else:
//                 lon1 = lon1buff.data[iii]
//                 lat1 = lat1buff.data[iii]
//                 lon2 = lon2buff.data[iii]
//                 lat2 = lat2buff.data[iii]
//             geod_inverse(
//                 &self._geod_geodesic,
//                 lat1, lon1, lat2, lon2,
//                 &ps12, &pazi1, &pazi2,
//             )

//             # by default (return_back_azimuth=True),
//             # forward azimuth needs to be flipped 180 degrees
//             # to match the (back azimuth) output of PROJ geod utilities.
//             if return_back_azimuth:
//                 pazi2 = _reverse_azimuth(pazi2, factor=180)
//             if radians:
//                 lon1buff.data[iii] = _DG2RAD * pazi1
//                 lat1buff.data[iii] = _DG2RAD * pazi2
//             else:
//                 lon1buff.data[iii] = pazi1
//                 lat1buff.data[iii] = pazi2
//             # write azimuth data into lon2 buffer
//             lon2buff.data[iii] = ps12



// @cython.boundscheck(False)
// @cython.wraparound(False)
// def _inv_point(
//     self,
//     object lon1in,
//     object lat1in,
//     object lon2in,
//     object lat2in,
//     bint radians=False,
//     bint return_back_azimuth=True,
// ):
//     """
//     Scalar optimized function
//     inverse transformation - return forward and back azimuth, plus distance
//     between an initial and terminus lat/lon pair.
//     if radians=True, lons/lats are radians instead of degree
//     """
//     cdef:
//         double pazi1
//         double pazi2
//         double ps12
//         double lon1 = lon1in
//         double lat1 = lat1in
//         double lon2 = lon2in
//         double lat2 = lat2in

//     # We do the type-checking internally here due to automatically
//     # casting length-1 arrays to float that we don't want to return scalar for.
//     # Ex: float(np.array([0])) works and we don't want to accept numpy arrays
//     for x_in in (lon1in, lat1in, lon2in, lat2in):
//         if not isinstance(x_in, (float, int)):
//             raise TypeError("Scalar input is required for point based functions")

//     with nogil:
//         if radians:
//             lon1 = _RAD2DG * lon1
//             lat1 = _RAD2DG * lat1
//             lon2 = _RAD2DG * lon2
//             lat2 = _RAD2DG * lat2
//         geod_inverse(
//             &self._geod_geodesic,
//             lat1, lon1, lat2, lon2,
//             &ps12, &pazi1, &pazi2,
//         )
//         # back azimuth needs to be flipped 180 degrees
//         # to match what proj4 geod utility produces.
//         if return_back_azimuth:
//             pazi2 =_reverse_azimuth(pazi2, factor=180)
//         if radians:
//             pazi1 = _DG2RAD * pazi1
//             pazi2 = _DG2RAD * pazi2
//     return pazi1, pazi2, ps12





// double _reverse_azimuth(double azi, double factor) nogil:
//     if azi > 0:
//         azi = azi - factor
//     else:
//         azi = azi + factor
//     return azi




// void geod_inverse(const struct geod_geodesic* g,
//                   double lat1, double lon1, double lat2, double lon2,
//                   double* ps12, double* pazi1, double* pazi2) {
//   geod_geninverse(g, lat1, lon1, lat2, lon2, ps12, pazi1, pazi2,
//                   NULLPTR, NULLPTR, NULLPTR, NULLPTR);
// }



// double geod_geninverse(const struct geod_geodesic* g,
//                        double lat1, double lon1, double lat2, double lon2,
//                        double* ps12, double* pazi1, double* pazi2,
//                        double* pm12, double* pM12, double* pM21,
//                        double* pS12) {
//   double salp1, calp1, salp2, calp2,
//     a12 = geod_geninverse_int(g, lat1, lon1, lat2, lon2, ps12,
//                               &salp1, &calp1, &salp2, &calp2,
//                               pm12, pM12, pM21, pS12);
//   if (pazi1) *pazi1 = atan2dx(salp1, calp1);
//   if (pazi2) *pazi2 = atan2dx(salp2, calp2);
//   return a12;
// }



// static double geod_geninverse_int(const struct geod_geodesic* g,
//                                   double lat1, double lon1,
//                                   double lat2, double lon2,
//                                   double* ps12,
//                                   double* psalp1, double* pcalp1,
//                                   double* psalp2, double* pcalp2,
//                                   double* pm12, double* pM12, double* pM21,
//                                   double* pS12) {
//   double s12 = 0, m12 = 0, M12 = 0, M21 = 0, S12 = 0;
//   double lon12, lon12s;
//   int latsign, lonsign, swapp;
//   double sbet1, cbet1, sbet2, cbet2, s12x = 0, m12x = 0;
//   double dn1, dn2, lam12, slam12, clam12;
//   double a12 = 0, sig12, calp1 = 0, salp1 = 0, calp2 = 0, salp2 = 0;
//   double Ca[nC];
//   boolx meridian;
//   /* somg12 == 2 marks that it needs to be calculated */
//   double omg12 = 0, somg12 = 2, comg12 = 0;

//   unsigned outmask =
//     (ps12 ? GEOD_DISTANCE : GEOD_NONE) |
//     (pm12 ? GEOD_REDUCEDLENGTH : GEOD_NONE) |
//     (pM12 || pM21 ? GEOD_GEODESICSCALE : GEOD_NONE) |
//     (pS12 ? GEOD_AREA : GEOD_NONE);

//   outmask &= OUT_ALL;
//   /* Compute longitude difference (AngDiff does this carefully).  Result is
//    * in [-180, 180] but -180 is only for west-going geodesics.  180 is for
//    * east-going and meridional geodesics. */
//   lon12 = AngDiff(lon1, lon2, &lon12s);
//   /* Make longitude difference positive. */
//   lonsign = signbit(lon12) ? -1 : 1;
//   lon12 *= lonsign; lon12s *= lonsign;
//   lam12 = lon12 * degree;
//   /* Calculate sincos of lon12 + error (this applies AngRound internally). */
//   sincosde(lon12, lon12s, &slam12, &clam12);
//   lon12s = (hd - lon12) - lon12s; /* the supplementary longitude difference */

//   /* If really close to the equator, treat as on equator. */
//   lat1 = AngRound(LatFix(lat1));
//   lat2 = AngRound(LatFix(lat2));
//   /* Swap points so that point with higher (abs) latitude is point 1
//    * If one latitude is a nan, then it becomes lat1. */
//   swapp = fabs(lat1) < fabs(lat2) || lat2 != lat2 ? -1 : 1;
//   if (swapp < 0) {
//     lonsign *= -1;
//     swapx(&lat1, &lat2);
//   }
//   /* Make lat1 <= -0 */
//   latsign = signbit(lat1) ? 1 : -1;
//   lat1 *= latsign;
//   lat2 *= latsign;
//   /* Now we have
//    *
//    *     0 <= lon12 <= 180
//    *     -90 <= lat1 <= -0
//    *     lat1 <= lat2 <= -lat1
//    *
//    * longsign, swapp, latsign register the transformation to bring the
//    * coordinates to this canonical form.  In all cases, 1 means no change was
//    * made.  We make these transformations so that there are few cases to
//    * check, e.g., on verifying quadrants in atan2.  In addition, this
//    * enforces some symmetries in the results returned. */

//   sincosdx(lat1, &sbet1, &cbet1); sbet1 *= g->f1;
//   /* Ensure cbet1 = +epsilon at poles */
//   norm2(&sbet1, &cbet1); cbet1 = fmax(tiny, cbet1);

//   sincosdx(lat2, &sbet2, &cbet2); sbet2 *= g->f1;
//   /* Ensure cbet2 = +epsilon at poles */
//   norm2(&sbet2, &cbet2); cbet2 = fmax(tiny, cbet2);

//   /* If cbet1 < -sbet1, then cbet2 - cbet1 is a sensitive measure of the
//    * |bet1| - |bet2|.  Alternatively (cbet1 >= -sbet1), abs(sbet2) + sbet1 is
//    * a better measure.  This logic is used in assigning calp2 in Lambda12.
//    * Sometimes these quantities vanish and in that case we force bet2 = +/-
//    * bet1 exactly.  An example where is is necessary is the inverse problem
//    * 48.522876735459 0 -48.52287673545898293 179.599720456223079643
//    * which failed with Visual Studio 10 (Release and Debug) */

//   if (cbet1 < -sbet1) {
//     if (cbet2 == cbet1)
//       sbet2 = copysign(sbet1, sbet2);
//   } else {
//     if (fabs(sbet2) == -sbet1)
//       cbet2 = cbet1;
//   }

//   dn1 = sqrt(1 + g->ep2 * sq(sbet1));
//   dn2 = sqrt(1 + g->ep2 * sq(sbet2));

//   meridian = lat1 == -qd || slam12 == 0;

//   if (meridian) {

//     /* Endpoints are on a single full meridian, so the geodesic might lie on
//      * a meridian. */

//     double ssig1, csig1, ssig2, csig2;
//     calp1 = clam12; salp1 = slam12; /* Head to the target longitude */
//     calp2 = 1; salp2 = 0;           /* At the target we're heading north */

//     /* tan(bet) = tan(sig) * cos(alp) */
//     ssig1 = sbet1; csig1 = calp1 * cbet1;
//     ssig2 = sbet2; csig2 = calp2 * cbet2;

//     /* sig12 = sig2 - sig1 */
//     sig12 = atan2(fmax(0.0, csig1 * ssig2 - ssig1 * csig2) + 0,
//                             csig1 * csig2 + ssig1 * ssig2);
//     Lengths(g, g->n, sig12, ssig1, csig1, dn1, ssig2, csig2, dn2,
//             cbet1, cbet2, &s12x, &m12x, NULLPTR,
//             (outmask & GEOD_GEODESICSCALE) ? &M12 : NULLPTR,
//             (outmask & GEOD_GEODESICSCALE) ? &M21 : NULLPTR,
//             Ca);
//     /* Add the check for sig12 since zero length geodesics might yield m12 <
//      * 0.  Test case was
//      *
//      *    echo 20.001 0 20.001 0 | GeodSolve -i
//      */
//     if (sig12 < tol2 || m12x >= 0) {
//       /* Need at least 2, to handle 90 0 90 180 */
//       if (sig12 < 3 * tiny ||
//           /* Prevent negative s12 or m12 for short lines */
//           (sig12 < tol0 && (s12x < 0 || m12x < 0)))
//         sig12 = m12x = s12x = 0;
//       m12x *= g->b;
//       s12x *= g->b;
//       a12 = sig12 / degree;
//     } else
//       /* m12 < 0, i.e., prolate and too close to anti-podal */
//       meridian = FALSE;
//   }

//   if (!meridian &&
//       sbet1 == 0 &&           /* and sbet2 == 0 */
//       /* Mimic the way Lambda12 works with calp1 = 0 */
//       (g->f <= 0 || lon12s >= g->f * hd)) {

//     /* Geodesic runs along equator */
//     calp1 = calp2 = 0; salp1 = salp2 = 1;
//     s12x = g->a * lam12;
//     sig12 = omg12 = lam12 / g->f1;
//     m12x = g->b * sin(sig12);
//     if (outmask & GEOD_GEODESICSCALE)
//       M12 = M21 = cos(sig12);
//     a12 = lon12 / g->f1;

//   } else if (!meridian) {

//     /* Now point1 and point2 belong within a hemisphere bounded by a
//      * meridian and geodesic is neither meridional or equatorial. */

//     /* Figure a starting point for Newton's method */
//     double dnm = 0;
//     sig12 = InverseStart(g, sbet1, cbet1, dn1, sbet2, cbet2, dn2,
//                          lam12, slam12, clam12,
//                          &salp1, &calp1, &salp2, &calp2, &dnm,
//                          Ca);

//     if (sig12 >= 0) {
//       /* Short lines (InverseStart sets salp2, calp2, dnm) */
//       s12x = sig12 * g->b * dnm;
//       m12x = sq(dnm) * g->b * sin(sig12 / dnm);
//       if (outmask & GEOD_GEODESICSCALE)
//         M12 = M21 = cos(sig12 / dnm);
//       a12 = sig12 / degree;
//       omg12 = lam12 / (g->f1 * dnm);
//     } else {

//       /* Newton's method.  This is a straightforward solution of f(alp1) =
//        * lambda12(alp1) - lam12 = 0 with one wrinkle.  f(alp) has exactly one
//        * root in the interval (0, pi) and its derivative is positive at the
//        * root.  Thus f(alp) is positive for alp > alp1 and negative for alp <
//        * alp1.  During the course of the iteration, a range (alp1a, alp1b) is
//        * maintained which brackets the root and with each evaluation of
//        * f(alp) the range is shrunk, if possible.  Newton's method is
//        * restarted whenever the derivative of f is negative (because the new
//        * value of alp1 is then further from the solution) or if the new
//        * estimate of alp1 lies outside (0,pi); in this case, the new starting
//        * guess is taken to be (alp1a + alp1b) / 2. */
//       double ssig1 = 0, csig1 = 0, ssig2 = 0, csig2 = 0, eps = 0, domg12 = 0;
//       unsigned numit = 0;
//       /* Bracketing range */
//       double salp1a = tiny, calp1a = 1, salp1b = tiny, calp1b = -1;
//       boolx tripn = FALSE;
//       boolx tripb = FALSE;
//       for (;; ++numit) {
//         /* the WGS84 test set: mean = 1.47, sd = 1.25, max = 16
//          * WGS84 and random input: mean = 2.85, sd = 0.60 */
//         double dv = 0,
//           v = Lambda12(g, sbet1, cbet1, dn1, sbet2, cbet2, dn2, salp1, calp1,
//                         slam12, clam12,
//                         &salp2, &calp2, &sig12, &ssig1, &csig1, &ssig2, &csig2,
//                         &eps, &domg12, numit < maxit1, &dv, Ca);
//         if (tripb ||
//             /* Reversed test to allow escape with NaNs */
//             !(fabs(v) >= (tripn ? 8 : 1) * tol0) ||
//             /* Enough bisections to get accurate result */
//             numit == maxit2)
//           break;
//         /* Update bracketing values */
//         if (v > 0 && (numit > maxit1 || calp1/salp1 > calp1b/salp1b))
//           { salp1b = salp1; calp1b = calp1; }
//         else if (v < 0 && (numit > maxit1 || calp1/salp1 < calp1a/salp1a))
//           { salp1a = salp1; calp1a = calp1; }
//         if (numit < maxit1 && dv > 0) {
//           double
//             dalp1 = -v/dv;
//           if (fabs(dalp1) < pi) {
//             double
//               sdalp1 = sin(dalp1), cdalp1 = cos(dalp1),
//               nsalp1 = salp1 * cdalp1 + calp1 * sdalp1;
//             if (nsalp1 > 0) {
//               calp1 = calp1 * cdalp1 - salp1 * sdalp1;
//               salp1 = nsalp1;
//               norm2(&salp1, &calp1);
//               /* In some regimes we don't get quadratic convergence because
//                * slope -> 0.  So use convergence conditions based on epsilon
//                * instead of sqrt(epsilon). */
//               tripn = fabs(v) <= 16 * tol0;
//               continue;
//             }
//           }
//         }
//         /* Either dv was not positive or updated value was outside legal
//          * range.  Use the midpoint of the bracket as the next estimate.
//          * This mechanism is not needed for the WGS84 ellipsoid, but it does
//          * catch problems with more eccentric ellipsoids.  Its efficacy is
//          * such for the WGS84 test set with the starting guess set to alp1 =
//          * 90deg:
//          * the WGS84 test set: mean = 5.21, sd = 3.93, max = 24
//          * WGS84 and random input: mean = 4.74, sd = 0.99 */
//         salp1 = (salp1a + salp1b)/2;
//         calp1 = (calp1a + calp1b)/2;
//         norm2(&salp1, &calp1);
//         tripn = FALSE;
//         tripb = (fabs(salp1a - salp1) + (calp1a - calp1) < tolb ||
//                  fabs(salp1 - salp1b) + (calp1 - calp1b) < tolb);
//       }
//       Lengths(g, eps, sig12, ssig1, csig1, dn1, ssig2, csig2, dn2,
//               cbet1, cbet2, &s12x, &m12x, NULLPTR,
//               (outmask & GEOD_GEODESICSCALE) ? &M12 : NULLPTR,
//               (outmask & GEOD_GEODESICSCALE) ? &M21 : NULLPTR, Ca);
//       m12x *= g->b;
//       s12x *= g->b;
//       a12 = sig12 / degree;
//       if (outmask & GEOD_AREA) {
//         /* omg12 = lam12 - domg12 */
//         double sdomg12 = sin(domg12), cdomg12 = cos(domg12);
//         somg12 = slam12 * cdomg12 - clam12 * sdomg12;
//         comg12 = clam12 * cdomg12 + slam12 * sdomg12;
//       }
//     }
//   }

//   if (outmask & GEOD_DISTANCE)
//     s12 = 0 + s12x;             /* Convert -0 to 0 */

//   if (outmask & GEOD_REDUCEDLENGTH)
//     m12 = 0 + m12x;             /* Convert -0 to 0 */

//   if (outmask & GEOD_AREA) {
//     double
//       /* From Lambda12: sin(alp1) * cos(bet1) = sin(alp0) */
//       salp0 = salp1 * cbet1,
//       calp0 = hypot(calp1, salp1 * sbet1); /* calp0 > 0 */
//     double alp12;
//     if (calp0 != 0 && salp0 != 0) {
//       double
//         /* From Lambda12: tan(bet) = tan(sig) * cos(alp) */
//         ssig1 = sbet1, csig1 = calp1 * cbet1,
//         ssig2 = sbet2, csig2 = calp2 * cbet2,
//         k2 = sq(calp0) * g->ep2,
//         eps = k2 / (2 * (1 + sqrt(1 + k2)) + k2),
//         /* Multiplier = a^2 * e^2 * cos(alpha0) * sin(alpha0). */
//         A4 = sq(g->a) * calp0 * salp0 * g->e2;
//       double B41, B42;
//       norm2(&ssig1, &csig1);
//       norm2(&ssig2, &csig2);
//       C4f(g, eps, Ca);
//       B41 = SinCosSeries(FALSE, ssig1, csig1, Ca, nC4);
//       B42 = SinCosSeries(FALSE, ssig2, csig2, Ca, nC4);
//       S12 = A4 * (B42 - B41);
//     } else
//       /* Avoid problems with indeterminate sig1, sig2 on equator */
//       S12 = 0;

//     if (!meridian && somg12 == 2) {
//       somg12 = sin(omg12); comg12 = cos(omg12);
//     }

//     if (!meridian &&
//         /* omg12 < 3/4 * pi */
//         comg12 > -0.7071 &&     /* Long difference not too big */
//         sbet2 - sbet1 < 1.75) { /* Lat difference not too big */
//       /* Use tan(Gamma/2) = tan(omg12/2)
//        * * (tan(bet1/2)+tan(bet2/2))/(1+tan(bet1/2)*tan(bet2/2))
//        * with tan(x/2) = sin(x)/(1+cos(x)) */
//       double
//         domg12 = 1 + comg12, dbet1 = 1 + cbet1, dbet2 = 1 + cbet2;
//       alp12 = 2 * atan2( somg12 * ( sbet1 * dbet2 + sbet2 * dbet1 ),
//                          domg12 * ( sbet1 * sbet2 + dbet1 * dbet2 ) );
//     } else {
//       /* alp12 = alp2 - alp1, used in atan2 so no need to normalize */
//       double
//         salp12 = salp2 * calp1 - calp2 * salp1,
//         calp12 = calp2 * calp1 + salp2 * salp1;
//       /* The right thing appears to happen if alp1 = +/-180 and alp2 = 0, viz
//        * salp12 = -0 and alp12 = -180.  However this depends on the sign
//        * being attached to 0 correctly.  The following ensures the correct
//        * behavior. */
//       if (salp12 == 0 && calp12 < 0) {
//         salp12 = tiny * calp1;
//         calp12 = -1;
//       }
//       alp12 = atan2(salp12, calp12);
//     }
//     S12 += g->c2 * alp12;
//     S12 *= swapp * lonsign * latsign;
//     /* Convert -0 to 0 */
//     S12 += 0;
//   }

//   /* Convert calp, salp to azimuth accounting for lonsign, swapp, latsign. */
//   if (swapp < 0) {
//     swapx(&salp1, &salp2);
//     swapx(&calp1, &calp2);
//     if (outmask & GEOD_GEODESICSCALE)
//       swapx(&M12, &M21);
//   }

//   salp1 *= swapp * lonsign; calp1 *= swapp * latsign;
//   salp2 *= swapp * lonsign; calp2 *= swapp * latsign;

//   if (psalp1) *psalp1 = salp1;
//   if (pcalp1) *pcalp1 = calp1;
//   if (psalp2) *psalp2 = salp2;
//   if (pcalp2) *pcalp2 = calp2;

//   if (outmask & GEOD_DISTANCE)
//     *ps12 = s12;
//   if (outmask & GEOD_REDUCEDLENGTH)
//     *pm12 = m12;
//   if (outmask & GEOD_GEODESICSCALE) {
//     if (pM12) *pM12 = M12;
//     if (pM21) *pM21 = M21;
//   }
//   if (outmask & GEOD_AREA)
//     *pS12 = S12;

//   /* Returned value in [0, 180] */
//   return a12;
// }
