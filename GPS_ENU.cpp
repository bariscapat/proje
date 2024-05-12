#include "GPS_ENU.h"
#include <SoftwareSerial.h>


const double a = 6378137.0;         // Earth's semi-major axis in meters
const double b = 6356752.314245;    // Earth's semi-minor axis in meters
const double f = (a - b) / a;       // Earth's flattening factor 
const double e1 = 0.00669437999013; // Square of eccentricity of WGS84 ellipsoid
const double e2 = 0.00673949674227; // Square of eccentricity of WGS84 ellipsoid

_ECEF_G GPS_ecef;
_ECEF_G GPSRef_ecef;
_ENU_G  GPS_enu;

_GEO_G  GPS_Out;
_GEO_G  GPS_Ref;

double limitEps(double val, const double eps)
{
    if(val >= 0 && val < eps)
        val = eps;
    else if(val <= 0 && val > -eps)
        val = -eps;
    return val;
}
void GEO_ECEF(double longitude, double latitude, double altitude)
{
    double cosLat = cos(latitude);
    double sinLat = sin(latitude);
    double cosLon = cos(longitude);
    double sinLon = sin(longitude);

    double N = a / sqrt(1.0 - e1 * sinLat * sinLat);

    GPS_ecef.x = (N + altitude) * cosLat * cosLon;
    GPS_ecef.y = (N + altitude) * cosLat * sinLon;
    GPS_ecef.z = (N * (1.0 - e1) + altitude) * sinLat;
}
void GEORef_ECEF(double longitude, double latitude, double altitude)
{
    double cosLat = cos(latitude);
    double sinLat = sin(latitude);
    double cosLon = cos(longitude);
    double sinLon = sin(longitude);

    double N = a / sqrt(1.0 - e1 * sinLat * sinLat);

    GPSRef_ecef.x = (N + altitude) * cosLat * cosLon;
    GPSRef_ecef.y = (N + altitude) * cosLat * sinLon;
    GPSRef_ecef.z = (N * (1.0 - e1) + altitude) * sinLat;
}
void ECEF_ENU()
{
    double phi = GPS_Ref.latitude;
    double lambda = GPS_Ref.longitude;

    double sinPhi = sin(phi);
    double cosPhi = cos(phi);
    double sinLambda = sin(lambda);
    double cosLambda = cos(lambda);

    double x0 = GPS_ecef.x - GPSRef_ecef.x;
    double y0 = GPS_ecef.y - GPSRef_ecef.y;
    double z0 = GPS_ecef.z - GPSRef_ecef.z;

    GPS_enu.east =  (-sinLambda          * x0) + (cosLambda          * y0) + (0.0    * z0);
    GPS_enu.north = (-cosLambda * sinPhi * x0) - (sinPhi * sinLambda * y0) + (cosPhi * z0);
    GPS_enu.up =    (cosPhi * cosLambda  * x0) + (cosPhi * sinLambda * y0) + (sinPhi * z0);
}
void ENU_ECEF(double east, double north, double up, double* x, double* y,double* z)
{
  double phi    = GPS_Ref.latitude;
  double lambda = GPS_Ref.longitude;

  double sinPhi = sin(phi);
  double cosPhi = cos(phi);
  double sinLambda = sin(lambda);
  double cosLambda = cos(lambda);

  *x = (-sinLambda * east) - (sinPhi * cosLambda * north) + (cosPhi * cosLambda * up);
  *y = (cosLambda  * east) - (sinPhi * sinLambda * north) + (cosPhi * sinLambda * up);
  *z = (0.0        * east) + (cosPhi             * north) + (sinPhi             * up);

  *x +=GPSRef_ecef.x;
  *y +=GPSRef_ecef.y;
  *z +=GPSRef_ecef.z;
}
void GPS_to_ENU_Update(double longitude, double latitude, double altitude)
{
    // Convert Long, Lat to radians
    longitude = longitude * M_PI / 180.0;
    latitude = latitude * M_PI / 180.0;

  GEO_ECEF(longitude, latitude, altitude);
  ECEF_ENU();
}
void ENU_to_GPS_Update(double east, double north, double up)
{
  double x, y, z;

  ENU_ECEF (east, north, up, &x, &y, &z);

  double p = sqrt((x * x) + (y * y));
  double r = sqrt((x * x) + (y * y) + (z * z));
  double u = atan2(z * a, b * p);

  GPS_Out.longitude = atan2(y, x);
  {
		GPS_Out.latitude = atan2(z + e2 * b * pow(sin(u),3) 
                            ,p - a * e1 * pow(cos(u),3));
		double N = a / (sqrt(1.0 - e1 * pow(sin(GPS_Out.latitude),2)));
    double cos_lat = limitEps(cos(GPS_Out.latitude),1e-5);

		GPS_Out.altitude = p / cos_lat - N;
  }

  GPS_Out.longitude*= (180.0 / M_PI);
  GPS_Out.latitude *= (180.0 / M_PI);
}
void Set_GPS_Ref(double longitude, double latitude, double altitude)
{
  GPS_Ref.longitude = longitude * M_PI / 180.0;
  GPS_Ref.latitude  = latitude  * M_PI / 180.0;
  GPS_Ref.altitude  = altitude                ;

  GEORef_ECEF(GPS_Ref.longitude, GPS_Ref.latitude, GPS_Ref.altitude); // Set Reference in ECEF
}

