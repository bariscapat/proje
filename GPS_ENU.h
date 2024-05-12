#ifndef GPS_ENU_H
#define GPS_ENU_H

#include <math.h>

typedef struct {
    double x;
    double y;
    double z;
} _ECEF_G;

typedef struct {
    double east;
    double north;
    double up;
} _ENU_G;

typedef struct {
    double longitude;
    double latitude;
    double altitude;
} _GEO_G;

void GPS_to_ENU_Update(double longitude, double latitude, double altitude);
void ENU_to_GPS_Update(double east, double north, double up);
void Set_GPS_Ref(double longitude, double latitude, double altitude);
void ENU_ECEF(double east, double north, double up, double* x, double* y,double* z);

#endif