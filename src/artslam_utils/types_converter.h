#ifndef ARTSLAM_CORE_TYPES_CONVERTER_H
#define ARTSLAM_CORE_TYPES_CONVERTER_H

#include <cmath>
#include <stdio.h>
#include <stdlib.h>

namespace artslam::core::utils {
    class TypesConverter {
    public:
        // Grid granularity for rounding UTM coordinates to generate MapXY.
        static constexpr double grid_size = 100000.0;           // 100 km grid

        // Angle conversions
        static constexpr double DEG_TO_RAD = M_PI / 180.0;      // degrees to radians
        static constexpr double RAD_TO_DEG = 180.0 / M_PI;      // radians to degrees

        // WGS84 Parameters
        static constexpr double WGS84_A = 6378137.0;		    // major axis
        static constexpr double WGS84_B = 6356752.31424518;	    // minor axis
        static constexpr double WGS84_F = 0.0033528107;		    // ellipsoid flattening
        static constexpr double WGS84_E = 0.0818191908;	        // first eccentricity
        static constexpr double WGS84_EP =	0.0820944379;       // second eccentricity

        // UTM Parameters
        static constexpr double UTM_K0	= 0.9996;			    // scale factor
        static constexpr double UTM_FE	= 500000.0;     	    // false easting
        static constexpr double UTM_FN_N = 0.0;                 // false northing, northern hemisphere
        static constexpr double UTM_FN_S = 10000000.0 ;         // false northing, southern hemisphere
        static constexpr double UTM_E2 = (WGS84_E*WGS84_E);	    // e^2
        static constexpr double UTM_E4 = (UTM_E2*UTM_E2);	    // e^4
        static constexpr double UTM_E6 = (UTM_E4*UTM_E2);	    // e^6
        static constexpr double UTM_EP2 = (UTM_E2/(1-UTM_E2));	// e'^2

        // Determines the correct UTM letter designator for the given latitude,
        // 'Z' if it is outside the UTM limits of 84N to 80S
        static char UTM_letter_designator(double latitude);

        // Convert latitude and longitude to UTM coordinates.
        // East Longitudes are positive, West longitudes are negative.
        // North latitudes are positive, South latitudes are negative
        // Latitude and longitude are in fractional degrees
        static void LL_to_UTM(double latitude, double longitude, double &UTMNorthing, double &UTMEasting, int &UTMZone, char &UTMBand);

        // Convert latitude and longitude to UTM coordinates.
        // East Longitudes are positive, West longitudes are negative.
        // North latitudes are positive, South latitudes are negative
        // Latitude and longitude are in fractional degrees
        static void UTM_to_LL(double UTMNorthing, double UTMEasting, const char* UTMZone, double& Lat,  double& Long );
    };
}


#endif //ARTSLAM_CORE_TYPES_CONVERTER_H
