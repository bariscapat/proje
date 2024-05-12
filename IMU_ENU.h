#ifndef IMU_ENU_H
#define IMU_ENU_H

#include <math.h>

typedef struct {
    double east;
    double north;
    double up;
} _ENU_IMU;

typedef struct {
    double AccelX;
    double AccelY;
    double AccelZ;
    double Roll;
    double Pitch;
    double Yaw;
} _ECEF_IMU;

void IMU_to_ENU_Update(double accelX, double accelY, double accelZ, 
                       double roll  , double pitch , double yaw  );

#endif