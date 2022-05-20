#include "types_converter.h"
#include <iostream>

// Determines the correct UTM letter designator for the given latitude,
// 'Z' if it is outside the UTM limits of 84N to 80S
namespace artslam::core::utils {
    char TypesConverter::UTM_letter_designator(double latitude) {
        char letter_designation;

        if ((84 >= latitude) && (latitude >= 72)) letter_designation = 'X';
        else if ((72 > latitude) && (latitude >= 64)) letter_designation = 'W';
        else if ((64 > latitude) && (latitude >= 56)) letter_designation = 'V';
        else if ((56 > latitude) && (latitude >= 48)) letter_designation = 'U';
        else if ((48 > latitude) && (latitude >= 40)) letter_designation = 'T';
        else if ((40 > latitude) && (latitude >= 32)) letter_designation = 'S';
        else if ((32 > latitude) && (latitude >= 24)) letter_designation = 'R';
        else if ((24 > latitude) && (latitude >= 16)) letter_designation = 'Q';
        else if ((16 > latitude) && (latitude >= 8)) letter_designation = 'P';
        else if ((8 > latitude) && (latitude >= 0)) letter_designation = 'N';
        else if ((0 > latitude) && (latitude >= -8)) letter_designation = 'M';
        else if ((-8 > latitude) && (latitude >= -16)) letter_designation = 'L';
        else if ((-16 > latitude) && (latitude >= -24)) letter_designation = 'K';
        else if ((-24 > latitude) && (latitude >= -32)) letter_designation = 'J';
        else if ((-32 > latitude) && (latitude >= -40)) letter_designation = 'H';
        else if ((-40 > latitude) && (latitude >= -48)) letter_designation = 'G';
        else if ((-48 > latitude) && (latitude >= -56)) letter_designation = 'F';
        else if ((-56 > latitude) && (latitude >= -64)) letter_designation = 'E';
        else if ((-64 > latitude) && (latitude >= -72)) letter_designation = 'D';
        else if ((-72 > latitude) && (latitude >= -80)) letter_designation = 'C';
        else letter_designation = 'Z';    // 'Z' is an error flag, the Latitude is outside the UTM limits
        return letter_designation;
    }

    void
    TypesConverter::LL_to_UTM(const double latitude, const double longitude, double &UTMNorthing, double &UTMEasting,
                              int &UTMZone, char &UTMBand) {
        double a = WGS84_A;
        double ecc_squared = UTM_E2;
        double k0 = UTM_K0;

        double longitude_origin;
        double ecc_prime_squared;
        double N, T, C, A, M;

        //Make sure the longitude is between -180.00 .. 179.9
        double longitude_tmp = (longitude + 180) - int((longitude + 180) / 360) * 360 - 180;

        double latitude_radians = latitude * DEG_TO_RAD;
        double longitude_radians = longitude_tmp * DEG_TO_RAD;
        double longitude_origin_radians;
        int zone_id;

        zone_id = int((longitude_tmp + 180) / 6) + 1;

        if (latitude >= 56.0 && latitude < 64.0 && longitude_tmp >= 3.0 && longitude_tmp < 12.0)
            zone_id = 32;

        // Special zones for Svalbard
        if (latitude >= 72.0 && latitude < 84.0) {
            if (longitude_tmp >= 0.0 && longitude_tmp < 9.0) zone_id = 31;
            else if (longitude_tmp >= 9.0 && longitude_tmp < 21.0) zone_id = 33;
            else if (longitude_tmp >= 21.0 && longitude_tmp < 33.0) zone_id = 35;
            else if (longitude_tmp >= 33.0 && longitude_tmp < 42.0) zone_id = 37;
        }
        // +3 puts origin in middle of zone
        longitude_origin = (zone_id - 1) * 6 - 180 + 3;
        longitude_origin_radians = longitude_origin * DEG_TO_RAD;

        UTMBand = UTM_letter_designator(latitude);
        std::cout << "MADDAI " << UTMBand << "\n";

        //compute the UTM Zone from the latitude and longitude
        //sprintf(UTMZone, "%d%c", zone_id, UTM_letter_designator(latitude));
        UTMZone = zone_id;

        ecc_prime_squared = (ecc_squared) / (1 - ecc_squared);

        N = a / sqrt(1 - ecc_squared * sin(latitude_radians) * sin(latitude_radians));
        T = tan(latitude_radians) * tan(latitude_radians);
        C = ecc_prime_squared * cos(latitude_radians) * cos(latitude_radians);
        A = cos(latitude_radians) * (longitude_radians - longitude_origin_radians);

        M = a * ((1 - ecc_squared / 4 - 3 * ecc_squared * ecc_squared / 64
                  - 5 * ecc_squared * ecc_squared * ecc_squared / 256) * latitude_radians
                 - (3 * ecc_squared / 8 + 3 * ecc_squared * ecc_squared / 32
                    + 45 * ecc_squared * ecc_squared * ecc_squared / 1024) * sin(2 * latitude_radians)
                 + (15 * ecc_squared * ecc_squared / 256
                    + 45 * ecc_squared * ecc_squared * ecc_squared / 1024) * sin(4 * latitude_radians)
                 - (35 * ecc_squared * ecc_squared * ecc_squared / 3072) * sin(6 * latitude_radians));

        UTMEasting = (double)
                (k0 * N * (A + (1 - T + C) * A * A * A / 6
                           + (5 - 18 * T + T * T + 72 * C - 58 * ecc_prime_squared) * A * A * A * A * A / 120)
                 + 500000.0);

        UTMNorthing = (double)
                (k0 * (M + N * tan(latitude_radians)
                           * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24
                              + (61 - 58 * T + T * T + 600 * C - 330 * ecc_prime_squared) * A * A * A * A * A * A /
                                720)));

        if (latitude < 0) {
            //10000000 meter offset for southern hemisphere
            UTMNorthing += 10000000.0;
        }
    }

    void TypesConverter::UTM_to_LL(const double UTMNorthing, const double UTMEasting, const char *UTMZone, double &Lat,
                                   double &Long) {
        {
            double k0 = UTM_K0;
            double a = WGS84_A;
            double eccSquared = UTM_E2;
            double eccPrimeSquared;
            double e1 = (1 - sqrt(1 - eccSquared)) / (1 + sqrt(1 - eccSquared));
            double N1, T1, C1, R1, D, M;
            double LongOrigin;
            double mu, phi1Rad;
            double x, y;
            int ZoneNumber;
            char *ZoneLetter;

            x = UTMEasting - 500000.0; //remove 500,000 meter offset for longitude
            y = UTMNorthing;

            ZoneNumber = strtoul(UTMZone, &ZoneLetter, 10);
            if ((*ZoneLetter - 'N') < 0) {
                //remove 10,000,000 meter offset used for southern hemisphere
                y -= 10000000.0;
            }

            //+3 puts origin in middle of zone
            LongOrigin = (ZoneNumber - 1) * 6 - 180 + 3;
            eccPrimeSquared = (eccSquared) / (1 - eccSquared);

            M = y / k0;
            mu = M / (a * (1 - eccSquared / 4 - 3 * eccSquared * eccSquared / 64
                           - 5 * eccSquared * eccSquared * eccSquared / 256));

            phi1Rad = mu + ((3 * e1 / 2 - 27 * e1 * e1 * e1 / 32) * sin(2 * mu)
                            + (21 * e1 * e1 / 16 - 55 * e1 * e1 * e1 * e1 / 32) * sin(4 * mu)
                            + (151 * e1 * e1 * e1 / 96) * sin(6 * mu));

            N1 = a / sqrt(1 - eccSquared * sin(phi1Rad) * sin(phi1Rad));
            T1 = tan(phi1Rad) * tan(phi1Rad);
            C1 = eccPrimeSquared * cos(phi1Rad) * cos(phi1Rad);
            R1 = a * (1 - eccSquared) / pow(1 - eccSquared * sin(phi1Rad) * sin(phi1Rad), 1.5);
            D = x / (N1 * k0);

            Lat = phi1Rad - ((N1 * tan(phi1Rad) / R1)
                             * (D * D / 2
                                - (5 + 3 * T1 + 10 * C1 - 4 * C1 * C1 - 9 * eccPrimeSquared) * D * D * D * D / 24
                                + (61 + 90 * T1 + 298 * C1 + 45 * T1 * T1 - 252 * eccPrimeSquared
                                   - 3 * C1 * C1) * D * D * D * D * D * D / 720));

            Lat = Lat * RAD_TO_DEG;

            Long = ((D - (1 + 2 * T1 + C1) * D * D * D / 6
                     + (5 - 2 * C1 + 28 * T1 - 3 * C1 * C1 + 8 * eccPrimeSquared + 24 * T1 * T1)
                       * D * D * D * D * D / 120)
                    / cos(phi1Rad));
            Long = LongOrigin + Long * RAD_TO_DEG;

        }
    }
}