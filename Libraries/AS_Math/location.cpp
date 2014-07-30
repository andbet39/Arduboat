#include <stdlib.h>
#include "AS_Math.h"
#include "AS_Common.h"

// radius of earth in meters
#define RADIUS_OF_EARTH 6378100

// scaling factor from 1e-7 degrees to meters at equater
// == 1.0e-7 * DEG_TO_RAD * RADIUS_OF_EARTH
#define LOCATION_SCALING_FACTOR 0.011131884502145034f
// inverse of LOCATION_SCALING_FACTOR
#define LOCATION_SCALING_FACTOR_INV 89.83204953368922f

float longitude_scale(const struct Location &loc)
{
    static int32_t last_lat;
    static float scale = 1.0;
    if (labs(last_lat - loc.lat) < 100000) {
        // we are within 0.01 degrees (about 1km) of the
        // same latitude. We can avoid the cos() and return
        // the same scale factor.
        return scale;
    }
    scale = cosf(loc.lat * 1.0e-7f * DEG_TO_RAD);
    last_lat = loc.lat;
    return scale;
}



// return distance in meters between two locations
float get_distance(const struct Location &loc1, const struct Location &loc2)
{
    float dlat              = (float)(loc2.lat - loc1.lat);
    float dlong             = ((float)(loc2.lng - loc1.lng)) * longitude_scale(loc2);
    return pythagorous2(dlat, dlong) * LOCATION_SCALING_FACTOR;
}

// return distance in centimeters to between two locations
uint32_t get_distance_cm(const struct Location &loc1, const struct Location &loc2)
{
    return get_distance(loc1, loc2) * 100;
}

// return bearing in centi-degrees between two locations
int32_t get_bearing_cd(const struct Location &loc1, const struct Location &loc2)
{
    int32_t off_x = loc2.lng - loc1.lng;
    int32_t off_y = (loc2.lat - loc1.lat) / longitude_scale(loc2);
    int32_t bearing = 9000 + atan2f(-off_y, off_x) * 5729.57795f;
    if (bearing < 0) bearing += 36000;
    return bearing;
}
