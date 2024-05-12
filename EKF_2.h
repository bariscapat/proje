#ifndef EKF2_H
#define EKF2_H

#include <BasicLinearAlgebra.h>
using namespace BLA;

#define Matrix_type double

#define Nstate 9   // No.of State Parameters
//  P_X, P_Y, P_Z, V_X, V_Y, V_Z, Roll, Pitch, Yaw

#define Nobs   6   // No. of Observable Parameters
// Pos_X, Pos_Y, Pos_Z, Roll, Pitch, Yaw

#define Ncom   3   // No. of Command
// Acc_X, Acc_Y, Acc_Z

// from analysis:
// Accx variance:0.015
// Accy variance:0.015
// Accy variance:0.06
// Long:1
// Lat:1
// Alt:4.35
// roll:2.25
// pitch:2.25
// yaw:1.75

// measurement std of the noise
#define n_p 0.2         // Position measurement noise
#define n_a 0.1         // Angle measurement noise
#define np2 (n_p*n_p)   // Squared position measurement noise Squared
#define na2 (n_a*n_a)   // Squared Angle measurement noise
#define npa (0)         // Squared Angle measurement noise

// process std (1/inertia)
#define m_p 0.05
#define m_s 0.05
#define m_a 0.001       // Acceleration noise from datasheet
#define mp2 (m_p*m_p)
#define ms2 (m_s*m_s)
#define ma2 (m_a*m_a)

//Delta time
#define dt  0.15
#define dt2 (0.5*dt*dt)

void Kalman_Init(void);	
void Kalman_Update(void);

#endif