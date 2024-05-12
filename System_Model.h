#ifndef SYSTEM_H
#define SYSTEM_H

struct _State
{
  float Pos_X = 0.f;
  float Pos_Y = 0.f;
  float Pos_Z = 0.f;
  float Vel_X = 0.f;
  float Vel_Y = 0.f;
  float Vel_Z = 0.f;
  float Roll  = 0.f;
  float Pitch = 0.f;
  float Yaw   = 0.f;
};
struct _Measure
{
  float Pos_X = 0.f;
  float Pos_Y = 0.f;
  float Pos_Z = 0.f;
  float Roll  = 0.f;
  float Pitch = 0.f;
  float Yaw   = 0.f;
};
struct _Command  // Control
{
  float Acc_X = 0.f;
  float Acc_Y = 0.f;
  float Acc_Z = 0.f;
};

#endif