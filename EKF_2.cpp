#include "EKF_2.h"
#include <SoftwareSerial.h>


/*
 * Implement the Kalman filter corresponding to the linear problem
 *    x_k = F*x_{k-1} + B*u_k + q_k   (evolution model)
 *    y_k = H*x_k + r_k               (measure)
*/

//INPUT MATRICES
    BLA::Matrix<Nstate,Nstate,Matrix_type> Id;      // Identity matrix
    BLA::Matrix<Nstate,Nstate,Matrix_type> F;       // State-time evolution matrix 
    BLA::Matrix<Nstate,Ncom,Matrix_type> B;         // Command matrix (optional) ( Control - Matrix )
    BLA::Matrix<Ncom,1,Matrix_type> Command;        // Command Vector 
    BLA::Matrix<Nstate,Nstate,Matrix_type> Q;       // Model covariance acting as (1/inertia)

    BLA::Matrix<Nobs,1,Matrix_type> observation;    // Measurement data from sensors
    BLA::Matrix<Nobs,Nstate,Matrix_type> H;         // Measurement matrix
    BLA::Matrix<Nstate,Nobs,Matrix_type> H_Tr;      // Measurement matrix Transpose
    BLA::Matrix<Nobs,Nobs,Matrix_type> R;           // Measurement noise covariance matrix

//OUTPUT MATRICES
    BLA::Matrix<Nstate,Nstate,Matrix_type> P;       // posterior covariance (do not modify, except to init!)
    BLA::Matrix<Nstate,1,Matrix_type> State;        // state vector (do not modify, except to init!)
    BLA::Matrix<Nstate,Nobs,Matrix_type> K;         // Kalman gain matrix


    
void Kalman_Init(void)
{
  Q.Fill(0.0);

  // x_k = F*x_{k-1} + B*u_k   (evolution model)
  // Time evolution matrix (Based on Kinametics equations)
  F = {
        1.0, 0.0, 0.0, dt , 0.0, 0.0, 0.0, 0.0, 0.0,
		    0.0, 1.0, 0.0, 0.0, dt , 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, dt , 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
		    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0
      };
  // Command evolution matrix (Based on Kinametics equations)
  B = {
        dt2, 0.0, 0.0, 
		    0.0, dt2, 0.0,
        0.0, 0.0, dt2,
        dt , 0.0, 0.0,
		    0.0, dt , 0.0,
        0.0, 0.0, dt ,
        0.0, 0.0, 0.0,
		    0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
      };
  // Model covariance matrix
  P = {
        mp2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		    0.0, mp2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, mp2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, ms2, 0.0, 0.0, 0.0, 0.0, 0.0,
		    0.0, 0.0, 0.0, 0.0, ms2, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, ms2, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ma2, 0.0, 0.0,
		    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ma2, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ma2
      };
  Q = {
        mp2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		    0.0, mp2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, mp2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, ms2, 0.0, 0.0, 0.0, 0.0, 0.0,
		    0.0, 0.0, 0.0, 0.0, ms2, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, ms2, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ma2, 0.0, 0.0,
		    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ma2, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ma2
      };

  // y_k = H*x_k + r_k  (measure)
  // Measurement matrix n the position (e.g. GPS) and Orientation (e.g. IMU)
  H = {
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
      };
  H_Tr = ~H ;
  // Measurement covariance matrix
  R = {
        np2, 0.0, 0.0, 0.0, 0.0, npa, 
        0.0, np2, 0.0, 0.0, npa, 0.0, 
        0.0, 0.0, np2, npa, 0.0, 0.0,
        0.0, 0.0, npa, na2, 0.0, 0.0,
        0.0, npa, 0.0, 0.0, na2, 0.0,
        npa, 0.0, 0.0, 0.0, 0.0, na2,
      };
  Id = {
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
		    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0
      };
}

void Kalman_Update()
{
  // ESTIMATION
  State = (F * State) + (B * Command);          // Estimation state ahead
  P = F * P * (~F) + Q;                         // Estimation the error covariance ahead

  // UPDATE
  K = P*(H_Tr)*(Inverse(H * P * (H_Tr) + R));   // Kalman Gain
  State += K*(observation - H * State);         // K*y  -- Update estimate with measurement 
  P = (Id - K * H ) * P;                        // update the error covariance
}

