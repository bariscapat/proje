#include "IMU_ENU.h"

_ENU_IMU  IMU_enu;

void IMU_to_ENU_Update(double accelX, double accelY, double accelZ, double roll, double pitch, double yaw)
{
    // Convert roll, pitch, and yaw to radians
    roll = roll * M_PI / 180.0;
    pitch = pitch * M_PI / 180.0;
    yaw = yaw * M_PI / 180.0;

    double sinRoll  = sin(roll);
    double cosRoll  = cos(roll);
    double sinPitch = sin(pitch);
    double cosPitch = cos(pitch);
    double sinYaw   = sin(yaw);
    double cosYaw   = cos(yaw);

    // Calculate the rotation matrix elements
    double R[3][3];
    R[0][0] =  cosYaw   * cosPitch;
    R[0][1] = -sinYaw   * cosRoll + cosYaw * sinPitch * sinRoll;
    R[0][2] =  sinYaw   * sinRoll + cosYaw * sinPitch * cosRoll;
    R[1][0] =  sinYaw   * cosPitch;
    R[1][1] =  cosYaw   * cosRoll + sinYaw * sinPitch * sinRoll;
    R[1][2] = -cosYaw   * sinRoll + sinYaw * sinPitch * cosRoll;
    R[2][0] = -sinPitch;
    R[2][1] =  cosPitch * sinRoll;
    R[2][2] =  cosPitch * cosRoll;

    // Apply the rotation matrix
    IMU_enu.east  = R[0][0] * accelX + R[0][1] * accelY + R[0][2] * accelZ;
    IMU_enu.north = R[1][0] * accelX + R[1][1] * accelY + R[1][2] * accelZ;
    IMU_enu.up    = R[2][0] * accelX + R[2][1] * accelY + R[2][2] * accelZ;
}
