
#include <SoftwareSerial.h>
#include <SPI.h>
#include <stdio.h>
#include "String.h"

#include <TinyGPSPlus.h>
#include <Adafruit_BMP085.h>
#include "MPU9250.h"
#include "SdFat.h"

#include "System_Model.h"

#include "EKF_2.h"
#include "GPS_ENU.h"
#include "IMU_ENU.h"

#define Calibration
#define SD_Card
  #define Logging_File     "NAV_LOG.txt"

//#define Testing
  #define Data_Source_File "test.txt"
  #define Data_Source_File_Samples 8770

////////////////////////IMU////////////////////////
  MPU9250 mpu; // The TinyGPSPlus object
  unsigned long IMU_Time = millis();
  static const uint32_t IMU_Sampling = 25; //sampling time in millisecond
  extern _ENU_IMU IMU_enu;
  //extern _ECEF_IMU IMU_ecef;
  typedef struct 
  {
    double AccX;
    double AccY;
    double AccZ;
    double GyroX;
    double GyroY;
    double GyroZ;
    double MagX;
    double MagY;
    double MagZ;   
  } MPU_CAL;
  MPU_CAL Calibrated ;
////////////////////////IMU////////////////////////
////////////////////////GPS////////////////////////
  static const int RXPin = 19, TXPin = 18;
  static const uint32_t GPSBaud = 9600;
  unsigned long GPS_Time = millis();
  static const uint32_t GPS_Sampling = 1000; //sampling time in millisecond
  TinyGPSPlus gps; // The TinyGPSPlus object
  extern _ENU_G  GPS_enu;
  extern _ECEF_G GPS_ecef;
  extern _ECEF_G GPSRef_ecef;
  bool First_GPS = 1;
  extern _GEO_G  GPS_Out;
  extern _GEO_G  GPS_Ref;
////////////////////////GPS////////////////////////
////////////////////////BMP////////////////////////
  Adafruit_BMP085 bmp; // The bmp object
  double BMP_ALT= 0;
////////////////////////BMP////////////////////////
////////////////////////SD/////////////////////////
 #ifdef SD_Card
  #define PinCS 53
  File myFile;          // The SD object
  SdFat SD;

  #ifdef Testing
    char SD_Buff [150] = {0};
    typedef struct 
    {
      uint32_t No_;

      float Long;
      char Long_str[15];
      float Lat;
      char Lat_str[15];
      float Alt;
      char Alt_str[15];
      float AccX;
      char AccX_str[15];
      float AccY;
      char AccY_str[15];
      float AccZ;
      char AccZ_str[15];
      float Roll;
      char Roll_str[15];
      float Pitch;
      char Pitch_str[15];
      float Yaw;
      char Yaw_str[15];
    } Sensor ; 

  Sensor SD_In;
  #else
    //char SD_Buff[512] = {0};
  #endif

 #endif
////////////////////////SD/////////////////////////
////////////////////////EKF////////////////////////
  bool Start_EKF = 0;
  extern BLA::Matrix<Nstate,1,Matrix_type> State;              // state vector (do not modify, except to init!)
  extern BLA::Matrix<Nobs,1,Matrix_type> observation;          // Measurement data from sensors
  extern BLA::Matrix<Ncom,1,Matrix_type> Command;              // Command Vector 
  bool First_Print = 0;
  bool First_Print_SD = 0;
////////////////////////EKF////////////////////////
////////////////////////LED////////////////////////
  const int LED_Yellow =  8;
  const int LED_Red    =  9;
  const int LED_Green  = 10;  

  bool LED_Yellow_State = HIGH;
  bool LED_Red_State    = HIGH;
  bool LED_Green_State  = HIGH;  

  #define Blink_Interval 1000
  unsigned long LED_Time = millis();
////////////////////////LED////////////////////////
void Toggle_LED (uint8_t LED, bool *LED_State, int Duration, uint8_t Count)
{
  digitalWrite(LED_Yellow, HIGH);
  digitalWrite(LED_Red, HIGH);
  digitalWrite(LED_Green, HIGH);
  LED_Yellow_State = HIGH;
  LED_Red_State    = HIGH;
  LED_Green_State  = HIGH;

  bool State = HIGH;
  for (int i = 0; i < Count ; i++)
  {
    digitalWrite(LED, !LED_State);
    delay(Duration);
  }

  digitalWrite(LED_Yellow, HIGH);
  digitalWrite(LED_Red, HIGH);
  digitalWrite(LED_Green, HIGH);
  LED_Yellow_State = HIGH;
  LED_Red_State    = HIGH;
  LED_Green_State  = HIGH;
  *LED_State       = HIGH;
}
void print_Everything() 
{
  #ifdef Testing
  ////////////////////Serial///////////////////////
    // // LLA ( Raw - Reference - Output )
    // Serial.print("LAT:");  
    // Serial.print(SD_In.Lat   ,11);
    // Serial.print(",LONG:"); 
    // Serial.print(SD_In.Long  ,11);
    // Serial.print(",ALT:");  
    // Serial.print(SD_In.Alt   ,6 );
    // // Serial.print(",ALT2:");              
    // // Serial.print(BMP_ALT,6);
    // Serial.print(",RefLAT:");  
    // Serial.print(GPS_Ref.latitude  ,11);
    // Serial.print(",RefLONG:"); 
    // Serial.print(GPS_Ref.longitude ,11);
    // Serial.print(",RefALT:");  
    // Serial.print(GPS_Ref.altitude  ,6);
    // Serial.print(",OutLAT:");  
    // Serial.print(GPS_Out.latitude  ,11);
    // Serial.print(",OutLONG:"); 
    // Serial.print(GPS_Out.longitude ,11);
    // Serial.print(",OutALT:");  
    // Serial.print(GPS_Out.altitude  ,6);

    // // IMU Print after calibration
    // Serial.print(",AccX:");  
    // Serial.print(SD_In.AccX ,6);
    // Serial.print(",AccY:");  
    // Serial.print(SD_In.AccY ,6);
    // Serial.print(",AccZ:");  
    // Serial.print(SD_In.AccZ ,6); 
    // Serial.print(",GyroX:");  
    // Serial.print(Calibrated.GyroX,6);
    // Serial.print(",GyroY:");  
    // Serial.print(Calibrated.GyroY,6);
    // Serial.print(",GyroZ:");  
    // Serial.print(Calibrated.GyroZ,6);
    // Serial.print(",MagX:");  
    // Serial.print(mpu.getMagX()   ,6);
    // Serial.print(",MagY:");  
    // Serial.print(mpu.getMagY()   ,6);
    // Serial.print(",MagZ:");  
    // Serial.print(mpu.getMagZ()   ,6);
    // Serial.print(",Roll:");  
    // Serial.print(SD_In.Roll   ,6);
    // Serial.print(",Pitch:");  
    // Serial.print(SD_In.Pitch  ,6);
    // Serial.print(",Yaw:");  
    // Serial.println(SD_In.Yaw  ,6);

    // // Current Location/Accel in ENU Frame
    // Serial.print(",E_PosE:");
    // Serial.print(GPS_enu.east,6);
    // Serial.print(",E_PosN:");
    // Serial.print(GPS_enu.north,6);
    // Serial.print(",E_PosU:");
    // Serial.print(GPS_enu.up,6);
    // Serial.print(",E_AccE:");
    // Serial.print(IMU_enu.east,6);
    // Serial.print(",E_AccN:");
    // Serial.print(IMU_enu.north,6);
    // Serial.print(",E_AccU:");
    // Serial.print(IMU_enu.up,6);

    // // Position/Velocity/Eular in ENU Frame After Fusion
    // Serial.print(",K_PosE:");  
    // Serial.print(State(0),6);
    // Serial.print(",K_PosN:"); 
    // Serial.print(State(1),6);
    // Serial.print(",K_PosU:");  
    // Serial.print(State(2),6);
    // Serial.print(",K_VelE:");  
    // Serial.print(State(3),6);
    // Serial.print(",K_VelN:"); 
    // Serial.print(State(4),6);
    // Serial.print(",K_VelU:");  
    // Serial.print(State(5),6);
    // Serial.print(",K_AccE:");  
    // Serial.print(State(6),6);
    // Serial.print(",K_AccN:"); 
    // Serial.print(State(7),6);
    // Serial.print(",K_AccU:");  
    // Serial.print(State(8),6);
    // Serial.print(",K_Roll:");  
    // Serial.print(State(9),6);
    // Serial.print(",K_Pitc:"); 
    // Serial.print(State(10),6);
    // Serial.print(",K_Yaw:" );  
    // Serial.println(State(11),6);

    // Serial.println(",");
  ////////////////////Serial///////////////////////
  /////////////////////CSV/////////////////////////
    if (!First_Print)
    {
      Serial.print("Time");                          // for Kalman time consumption debug
      // LLA ( Raw - Reference - Output )
      Serial.print(",LAT");  
      Serial.print(",LONG"); 
      Serial.print(",ALT");  
      Serial.print(",ALT2");              
      Serial.print(",RefLAT");  
      Serial.print(",RefLONG"); 
      Serial.print(",RefALT");  
      Serial.print(",OutLAT");  
      Serial.print(",OutLONG"); 
      Serial.print(",OutALT");  

      // IMU Print after calibration
      Serial.print(",AccX");  
      Serial.print(",AccY");  
      Serial.print(",AccZ");  
      Serial.print(",GyroX");  
      Serial.print(",GyroY");  
      Serial.print(",GyroZ");  
      Serial.print(",MagX");  
      Serial.print(",MagY");  
      Serial.print(",MagZ");  
      Serial.print(",Roll");  
      Serial.print(",Pitch");  
      Serial.print(",Yaw");  

      // Current Location/Accel in ENU Frame
      Serial.print(",E_PosE");
      Serial.print(",E_PosN");
      Serial.print(",E_PosU");
      Serial.print(",E_AccE");
      Serial.print(",E_AccN");
      Serial.print(",E_AccU");

      // Position/Velocity/Eular in ENU Frame After Fusion
      Serial.print(",K_PosE");  
      Serial.print(",K_PosN"); 
      Serial.print(",K_PosU");  
      Serial.print(",K_VelE");  
      Serial.print(",K_VelN"); 
      Serial.print(",K_VelU");  
      Serial.print(",K_Roll");  
      Serial.print(",K_Pitc"); 
      Serial.print(",K_Yaww");  

      Serial.println(",");

      First_Print = 1;
    } 
    else 
    {
      Serial.print(millis());                     // for Kalman time consumption debug

      // LLA ( Raw - Reference - Output )
      Serial.print(",");  
      Serial.print(SD_In.Lat   ,11);
      Serial.print(","); 
      Serial.print(SD_In.Long  ,11);
      Serial.print(",");  
      Serial.print(SD_In.Alt   ,6);
      Serial.print(",");              
      Serial.print(BMP_ALT,6);
      Serial.print(",");  
      Serial.print(GPS_Ref.latitude  ,11);
      Serial.print(","); 
      Serial.print(GPS_Ref.longitude ,11);
      Serial.print(",");  
      Serial.print(GPS_Ref.altitude  ,6);
      Serial.print(",");  
      Serial.print(GPS_Out.latitude    ,11);
      Serial.print(","); 
      Serial.print(GPS_Out.longitude   ,11);
      Serial.print(",");  
      Serial.print(GPS_Out.altitude  ,6);

      //IMU Print after calibration
      Serial.print(",");  
      Serial.print(SD_In.AccX ,6);
      Serial.print(",");  
      Serial.print(SD_In.AccY ,6);
      Serial.print(",");  
      Serial.print(SD_In.AccZ ,6); 
      Serial.print(",");  
      Serial.print(Calibrated.GyroX,6);
      Serial.print(",");  
      Serial.print(Calibrated.GyroY,6);
      Serial.print(",");  
      Serial.print(Calibrated.GyroZ,6);
      Serial.print(",");  
      Serial.print(mpu.getMagX()   ,6);
      Serial.print(",");  
      Serial.print(mpu.getMagY()   ,6);
      Serial.print(",");  
      Serial.print(mpu.getMagZ()   ,6);
      Serial.print(",");  
      Serial.print(SD_In.Roll   ,6);
      Serial.print(",");  
      Serial.print(SD_In.Pitch  ,6);
      Serial.print(",");  
      Serial.print(SD_In.Yaw    ,6);

      //Current Location/Accel in ENU Frame
      Serial.print(",");
      Serial.print(GPS_enu.east,6);
      Serial.print(",");
      Serial.print(GPS_enu.north,6);
      Serial.print(",");
      Serial.print(GPS_enu.up,6);
      Serial.print(",");
      Serial.print(IMU_enu.east,6);
      Serial.print(",");
      Serial.print(IMU_enu.north,6);
      Serial.print(",");
      Serial.print(IMU_enu.up,6);

      //Position/Velocity/Eular in ENU Frame After Fusion
      Serial.print(",");  
      Serial.print(State(0),6);
      Serial.print(","); 
      Serial.print(State(1),6);
      Serial.print(",");  
      Serial.print(State(2),6);
      Serial.print(",");  
      Serial.print(State(3),6);
      Serial.print(","); 
      Serial.print(State(4),6);
      Serial.print(",");  
      Serial.print(State(5),6);
      Serial.print(",");  
      Serial.print(State(6),6);
      Serial.print(","); 
      Serial.print(State(7),6);
      Serial.print("," );  
      Serial.print(State(8),6);
      Serial.println(",");
    }
  /////////////////////CSV/////////////////////////
  #else
  ////////////////////Serial///////////////////////
    // // LLA ( Raw - Reference - Output )
    // Serial.print("LAT:");  
    // Serial.print(gps.location.lat()   ,11);
    // Serial.print(",LONG:"); 
    // Serial.print(gps.location.lng()   ,11);
    // Serial.print(",ALT:");  
    // Serial.print(BMP_ALT               ,6);
    // Serial.print(",RefLAT:");  
    // Serial.print(GPS_Ref.latitude  ,11);
    // Serial.print(",RefLONG:"); 
    // Serial.print(GPS_Ref.longitude ,11);
    // Serial.print(",RefALT:");  
    // Serial.print(GPS_Ref.altitude  ,6);
    // Serial.print(",OutLAT:");  
    // Serial.print(GPS_Out.latitude  ,11);
    // Serial.print(",OutLONG:"); 
    // Serial.print(GPS_Out.longitude ,11);
    // Serial.print(",OutALT:");  
    // Serial.print(GPS_Out.altitude  ,6);

    // // IMU Print after calibration
    // Serial.print(",AccX:");  
    // Serial.print(Calibrated.AccX ,6);
    // Serial.print(",AccY:");  
    // Serial.print(Calibrated.AccY ,6);
    // Serial.print(",AccZ:");  
    // Serial.print(Calibrated.AccZ ,6); 
    // Serial.print(",GyroX:");  
    // Serial.print(Calibrated.GyroX,6);
    // Serial.print(",GyroY:");  
    // Serial.print(Calibrated.GyroY,6);
    // Serial.print(",GyroZ:");  
    // Serial.print(Calibrated.GyroZ,6);
    // Serial.print(",MagX:");  
    // Serial.print(mpu.getMagX()   ,6);
    // Serial.print(",MagY:");  
    // Serial.print(mpu.getMagY()   ,6);
    // Serial.print(",MagZ:");  
    // Serial.print(mpu.getMagZ()   ,6);
    // Serial.print(",Roll:");  
    // Serial.print(mpu.getRoll()   ,6);
    // Serial.print(",Pitch:");  
    // Serial.print(mpu.getPitch()  ,6);
    // Serial.print(",Yaw:");  
    // Serial.println(mpu.getYaw()  ,6);

    // // Current Location/Accel in ENU Frame
    // Serial.print(",E_PosE:");
    // Serial.print(GPS_enu.east,6);
    // Serial.print(",E_PosN:");
    // Serial.print(GPS_enu.north,6);
    // Serial.print(",E_PosU:");
    // Serial.print(GPS_enu.up,6);
    // Serial.print(",E_AccE:");
    // Serial.print(IMU_enu.east,6);
    // Serial.print(",E_AccN:");
    // Serial.print(IMU_enu.north,6);
    // Serial.print(",E_AccU:");
    // Serial.print(IMU_enu.up,6);

    // // Position/Velocity/Eular in ENU Frame After Fusion
    // Serial.print(",K_PosE:");  
    // Serial.print(State(0),6);
    // Serial.print(",K_PosN:"); 
    // Serial.print(State(1),6);
    // Serial.print(",K_PosU:");  
    // Serial.print(State(2),6);
    // Serial.print(",K_VelE:");  
    // Serial.print(State(3),6);
    // Serial.print(",K_VelN:"); 
    // Serial.print(State(4),6);
    // Serial.print(",K_VelU:");  
    // Serial.print(State(5),6);
    // Serial.print(",K_AccE:");  
    // Serial.print(State(6),6);
    // Serial.print(",K_AccN:"); 
    // Serial.print(State(7),6);
    // Serial.print(",K_AccU:");  
    // Serial.print(State(8),6);
    // Serial.print(",K_Roll:");  
    // Serial.print(State(9),6);
    // Serial.print(",K_Pitc:"); 
    // Serial.print(State(10),6);
    // Serial.print(",K_Yaw:" );  
    // Serial.println(State(11),6);

    // Serial.println(",");
  ////////////////////Serial///////////////////////
  /////////////////////CSV/////////////////////////
    if (!First_Print)
    {
      Serial.print("Time");                          // for Kalman time consumption debug
      // LLA ( Raw - Reference - Output )
      Serial.print(",LAT");  
      Serial.print(",LONG"); 
      Serial.print(",ALT");  
      Serial.print(",RefLAT");  
      Serial.print(",RefLONG"); 
      Serial.print(",RefALT");  
      Serial.print(",OutLAT");  
      Serial.print(",OutLONG"); 
      Serial.print(",OutALT");  

      // IMU Print after calibration
      Serial.print(",AccX");  
      Serial.print(",AccY");  
      Serial.print(",AccZ");  
      Serial.print(",GyroX");  
      Serial.print(",GyroY");  
      Serial.print(",GyroZ");  
      Serial.print(",MagX");  
      Serial.print(",MagY");  
      Serial.print(",MagZ");  
      Serial.print(",Roll");  
      Serial.print(",Pitch");  
      Serial.print(",Yaw");  

      // Current Location/Accel in ENU Frame
      Serial.print(",E_PosE");
      Serial.print(",E_PosN");
      Serial.print(",E_PosU");
      Serial.print(",E_AccE");
      Serial.print(",E_AccN");
      Serial.print(",E_AccU");

      // Position/Velocity/Eular in ENU Frame After Fusion
      Serial.print(",K_PosE");  
      Serial.print(",K_PosN"); 
      Serial.print(",K_PosU");  
      Serial.print(",K_VelE");  
      Serial.print(",K_VelN"); 
      Serial.print(",K_VelU");  
      Serial.print(",K_Roll");  
      Serial.print(",K_Pitc"); 
      Serial.print(",K_Yaww");  

      Serial.println(",");

      First_Print = 1;
    } 
    else 
    {
      Serial.print(millis());                     // for Kalman time consumption debug

      // LLA ( Raw - Reference - Output )
      Serial.print(",");  
      Serial.print(gps.location.lat()   ,11);
      Serial.print(","); 
      Serial.print(gps.location.lng()   ,11);
      Serial.print(",");               
      Serial.print(BMP_ALT,6);
      Serial.print(",");  
      Serial.print(GPS_Ref.latitude  ,11);
      Serial.print(","); 
      Serial.print(GPS_Ref.longitude ,11);
      Serial.print(",");  
      Serial.print(GPS_Ref.altitude  ,6);
      Serial.print(",");  
      Serial.print(GPS_Out.latitude    ,11);
      Serial.print(","); 
      Serial.print(GPS_Out.longitude   ,11);
      Serial.print(",");  
      Serial.print(GPS_Out.altitude  ,6);

      //IMU Print after calibration
      Serial.print(",");  
      Serial.print(Calibrated.AccX ,6);
      Serial.print(",");  
      Serial.print(Calibrated.AccY ,6);
      Serial.print(",");  
      Serial.print(Calibrated.AccZ ,6); 
      Serial.print(",");  
      Serial.print(Calibrated.GyroX,6);
      Serial.print(",");  
      Serial.print(Calibrated.GyroY,6);
      Serial.print(",");  
      Serial.print(Calibrated.GyroZ,6);
      Serial.print(",");  
      Serial.print(mpu.getMagX()   ,6);
      Serial.print(",");  
      Serial.print(mpu.getMagY()   ,6);
      Serial.print(",");  
      Serial.print(mpu.getMagZ()   ,6);
      Serial.print(",");  
      Serial.print(mpu.getRoll()   ,6);
      Serial.print(",");  
      Serial.print(mpu.getPitch()  ,6);
      Serial.print(",");  
      Serial.print(mpu.getYaw()    ,6);

      //Current Location/Accel in ENU Frame
      Serial.print(",");
      Serial.print(GPS_enu.east,6);
      Serial.print(",");
      Serial.print(GPS_enu.north,6);
      Serial.print(",");
      Serial.print(GPS_enu.up,6);
      Serial.print(",");
      Serial.print(IMU_enu.east,6);
      Serial.print(",");
      Serial.print(IMU_enu.north,6);
      Serial.print(",");
      Serial.print(IMU_enu.up,6);

      //Position/Velocity/Eular in ENU Frame After Fusion
      Serial.print(",");  
      Serial.print(State(0),6);
      Serial.print(","); 
      Serial.print(State(1),6);
      Serial.print(",");  
      Serial.print(State(2),6);
      Serial.print(",");  
      Serial.print(State(3),6);
      Serial.print(","); 
      Serial.print(State(4),6);
      Serial.print(",");  
      Serial.print(State(5),6);
      Serial.print(",");  
      Serial.print(State(6),6);
      Serial.print(","); 
      Serial.print(State(7),6);
      Serial.print("," );  
      Serial.print(State(8),6);
      Serial.println(",");
    }
  /////////////////////CSV/////////////////////////
  #endif
}
void print_State() 
{
  Serial.print("K_PosX:");  
  Serial.print(State(0));
  Serial.print(",K_PosY:"); 
  Serial.print(State(1));
  Serial.print(",K_PosZ:");  
  Serial.print(State(2));
  Serial.print(",K_VelX:");  
  Serial.print(State(3));
  Serial.print(",K_VelY:"); 
  Serial.print(State(4));
  Serial.print(",K_VelZ:");  
  Serial.print(State(5));
  Serial.print(",K_Roll:");  
  Serial.print(State(6));
  Serial.print(",K_Pitch:"); 
  Serial.print(State(7));
  Serial.print(",K_Yaw:");  
  Serial.println(State(8));
}
void SD_Push() 
{
  // #ifdef SD_Card
  //   if (myFile) 
  //   {
  //     myFile.print(millis());                     // for Kalman time consumption debug
  //     myFile.print(",");                          // for Kalman time consumption debug
  //     myFile.print("Longitude:");  
  //     myFile.print(State(0));
  //     myFile.print(",Latitude:"); 
  //     myFile.print(State(1));
  //     myFile.print(",Altitude:");  
  //     myFile.print(State(2));
  //     myFile.print(",Vel_East:");  
  //     myFile.print(State(3));
  //     myFile.print(",Vel_North:"); 
  //     myFile.print(State(4));
  //     myFile.print(",Vel_Up:");  
  //     myFile.print(State(5));
  //     myFile.print(",Roll:");  
  //     myFile.print(State(6));
  //     myFile.print(",Pitch:"); 
  //     myFile.print(State(7));
  //     myFile.print(",Yaw:");  
  //     myFile.println(State(8));
  //   }
  //   #endif
  //  SD_Buff.clear();
   char SD_Buff[150] = {0};
   myFile = SD.open(Logging_File, FILE_WRITE);  

    if (myFile && !First_Print_SD) 
    {
      //Serial.println("First Print");

      myFile.print(" OutLAT" );  
      myFile.print(",OutLONG"); 
      myFile.print(",OutALT" );
      myFile.print(",K_VelE" );  
      myFile.print(",K_VelN" ); 
      myFile.print(",K_VelU" );  
      myFile.print(",K_Roll" );  
      myFile.print(",K_Pitc" ); 
      myFile.println(",K_Yaww" ); 
      // strcat(SD_Buff," OutLAT" );  
      // strcat(SD_Buff,",OutLONG"); 
      // strcat(SD_Buff,",OutALT" );
      // strcat(SD_Buff,",K_VelE" );  
      // strcat(SD_Buff,",K_VelN" ); 
      // strcat(SD_Buff,",K_VelU" );  
      // strcat(SD_Buff,",K_Roll" );  
      // strcat(SD_Buff,",K_Pitc" ); 
      // strcat(SD_Buff,",K_Yaww" ); 
      //myFile.println(SD_Buff);
      First_Print_SD = 1;   
    }
    else if (myFile && First_Print_SD)
    {
      //sprintf(SD_Buff, "%.11f,%.11f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f", GPS_Out.latitude, GPS_Out.longitude, GPS_Out.altitude, State(3), State(3), State(3), State(3),State(3),State(3));
      // strcat(SD_Buff, ftoa(GPS_Out.latitude,11) );
      //Serial.println("Second Print");

      myFile.print(GPS_Out.latitude    ,11);
      myFile.write(","); 
      myFile.print(GPS_Out.longitude   ,11);
      myFile.write(",");  
      myFile.print(GPS_Out.altitude  ,6);
      myFile.write(",");  
      myFile.print(State(3),6);
      myFile.write(","); 
      myFile.print(State(4),6);
      myFile.write(",");  
      myFile.print(State(5),6);
      myFile.write(",");  
      myFile.print(State(6),6);
      myFile.write(","); 
      myFile.print(State(7),6);
      myFile.write("," );  
      myFile.println(State(8),6);
      //myFile.println(SD_Buff);
    }
    else 
      Serial.println("File error");

      myFile.close();

}
void Calibrate_IMU_RawData()
{
  Calibrated.AccX = (mpu.getAccX() - (mpu.getAccBiasX() / (float)MPU9250::CALIB_ACCEL_SENSITIVITY)) * 9.8;       // To remove calibration offset - change the result unit from g to m/sec2
  if (abs(Calibrated.AccX) < 0.1) 
    Calibrated.AccX = 0;
  Calibrated.AccY = (mpu.getAccY() - (mpu.getAccBiasY() / (float)MPU9250::CALIB_ACCEL_SENSITIVITY)) * 9.8;       // To remove calibration offset
  if (abs(Calibrated.AccY) < 0.1) 
    Calibrated.AccY = 0;
  Calibrated.AccZ = (mpu.getLinearAccZ() - (mpu.getAccBiasZ() / (float)MPU9250::CALIB_ACCEL_SENSITIVITY)) * 9.8; // To neglect gravity
   if (abs(Calibrated.AccZ) < 0.1) 
    Calibrated.AccZ = 0;

  Calibrated.GyroX = mpu.getGyroX() - (mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);    // To remove calibration offset
  if (abs(Calibrated.GyroX) < 0.2) 
    Calibrated.GyroX = 0;
  Calibrated.GyroY = mpu.getGyroY() - (mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);    // To remove calibration offset
  if (abs(Calibrated.GyroY) < 0.2) 
    Calibrated.GyroY = 0;
  Calibrated.GyroZ = mpu.getGyroZ() - (mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);    // To remove calibration offset
  if (abs(Calibrated.GyroZ) < 0.2) 
    Calibrated.GyroZ = 0;

  Calibrated.MagX = mpu.getMagX();
  Calibrated.MagY = mpu.getMagY();
  Calibrated.MagZ = mpu.getMagZ();
}
//////////////////////////////////////////////////////
void setup() 
{
    Serial.begin(500000);
    Serial.print("*********Start*********\r\n");
    Kalman_Init();
    pinMode(LED_Yellow, OUTPUT);
    pinMode(LED_Red   , OUTPUT);
    pinMode(LED_Green , OUTPUT);

    digitalWrite(LED_Yellow, LED_Yellow_State);
    digitalWrite(LED_Red   , LED_Red_State   );
    digitalWrite(LED_Green , LED_Green_State );

  ////////////////////////SD/////////////////////////
   #ifdef SD_Card
    if (!SD.begin(PinCS)) 
    {
      Serial.println("SD initialization failed!");
      LED_Red_State = LOW;
      digitalWrite(LED_Red, LED_Red_State);
      while (1);
    }
    Serial.println("SD initialization done.");
    Toggle_LED(LED_Yellow, &LED_Yellow_State, 500, 3);

    #ifdef Testing
      myFile = SD.open(Data_Source_File);
    #else
      myFile = SD.open(Logging_File, FILE_WRITE);  
    #endif
   #endif

  ////////////////////////SD///////////////////////// 
  ////////////////////////IMU//////////////////////// 
    Wire.begin();
    delay(2000);
    
    MPU9250Setting setting;
    setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
    setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
    setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
    setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
    setting.gyro_fchoice = 0x03;
    setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
    setting.accel_fchoice = 0x01;
    setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

    if (!mpu.setup(0x68,setting)) 
    {  // change to your own address
      while (1) 
      {        
        LED_Red_State = LOW;
        digitalWrite(LED_Red, LED_Red_State);
        Serial.println("MPU connection failed");
        delay(1000);
      }
    }
    #ifdef Calibration
    mpu.verbose(true);
    Serial.println("Accel Gyro calibration will start.");

    LED_Yellow_State = LOW;
    digitalWrite(LED_Yellow, LED_Yellow_State);
    
    Serial.println("Please leave the device still on the flat plane.");
    delay(3000);
    mpu.calibrateAccelGyro();
    Toggle_LED(LED_Green, &LED_Green_State, 500, 3);

    LED_Red_State = LOW;
    digitalWrite(LED_Red, LED_Red_State);
    Serial.println("Accel Gyro calibration is finished.");
    delay(5000);
    mpu.calibrateMag();
    mpu.verbose(false);
    Toggle_LED(LED_Green, &LED_Green_State, 500, 3);
    #endif
  ////////////////////////IMU////////////////////////
  ////////////////////////GPS////////////////////////
     Serial1.begin(GPSBaud);
     //GPS_Time = millis();
     //delay(1000);
  ////////////////////////GPS////////////////////////
  ////////////////////////BMP////////////////////////
    if (!bmp.begin()) 
    {
      Serial.println("Could not find a valid BMP085 sensor, check wiring!");
      while (1) {}
    }
  ////////////////////////BMP////////////////////////
}
//////////////////////////////////////////////////////
void loop() 
{
  #ifdef Testing
    //////////////////////Testing-SD///////////////////
      memset(SD_Buff, 0, sizeof(SD_Buff));
      char c ;
      uint32_t Buff_Count = 0;
      if (myFile)
      {
        // Serial.println("File Ok");
        // Line Scan from file on Sd-Card 
        do
        {
          c = myFile.read();
          SD_Buff[Buff_Count] = c; 
          Buff_Count ++;
        }while(c != '\n');

        // Samples Line Parse 
        uint32_t ccc =  sscanf (SD_Buff, "%u %s %s %s %s %s %s %s %s %s" ,&SD_In.No_,&SD_In.Lat_str,&SD_In.Long_str,&SD_In.Alt_str,&SD_In.AccX_str,&SD_In.AccY_str,&SD_In.AccZ_str,&SD_In.Roll_str,&SD_In.Pitch_str,&SD_In.Yaw_str);
        //Serial.println(SD_Buff);

        // Samples Parameters Conversion to Float 
        SD_In.Lat  = atof(SD_In.Lat_str );
        SD_In.Long = atof(SD_In.Long_str);
        SD_In.Alt  = atof(SD_In.Alt_str );
        SD_In.AccX = atof(SD_In.AccX_str);
        SD_In.AccY = atof(SD_In.AccY_str);
        SD_In.AccZ = atof(SD_In.AccZ_str);
        SD_In.Roll = atof(SD_In.Roll_str);
        SD_In.Pitch= atof(SD_In.Pitch_str);
        SD_In.Yaw  = atof(SD_In.Yaw_str );

        if (SD_In.No_ == Data_Source_File_Samples)
        {
          Serial.println("Finito");
          while (1){};
        }
      }
      else 
        Serial.println("File Error");
    //////////////////////Testing-SD///////////////////
    /////////////////////UpdateFilter//////////////////
      if (First_GPS && SD_In.Lat != 0)
      {
        Set_GPS_Ref(SD_In.Long, SD_In.Lat , SD_In.Alt); 
        First_GPS = 0;
      }
      GPS_to_ENU_Update(SD_In.Long, SD_In.Lat , SD_In.Alt);   // Needs Logitude and latitude in degrees - altitude in meters
      observation(0)=GPS_enu.east;    //GPS_ENU position in East axis (meters)
      observation(1)=GPS_enu.north;   //GPS_ENU position in North axis (meters)
      observation(2)=GPS_enu.up;      //GPS_ENU position in Up axis (meters)
      GPS_Time = millis();

      IMU_to_ENU_Update(SD_In.AccX, SD_In.AccY , SD_In.AccZ, 
                        SD_In.Roll, SD_In.Pitch, SD_In.Yaw );
      observation(3)=SD_In.Roll;        // Roll
      observation(4)=SD_In.Pitch;       // Pitch
      observation(5)=SD_In.Yaw;         // Yaw
      Command(0)=IMU_enu.east ;         // Acceleration in East axis direction  (meter/second^2)
      Command(1)=IMU_enu.north;         // Acceleration in North axis direction (meter/second^2)
      Command(2)=IMU_enu.up   ;         // Acceleration in Up axis direction    (meter/second^2)

      Kalman_Update();
      ENU_to_GPS_Update (State(0), State(1), State(2));  // Convert output back to Geodetic Frame
      print_Everything();
    /////////////////////UpdateFilter//////////////////
  #else
    ////////////////////////GPS+ALT////////////////////
      if (millis() - GPS_Time > 1000)
      {
        if (First_GPS && gps.location.lng() != 0)
        {
          Set_GPS_Ref(gps.location.lng(), gps.location.lat() , BMP_ALT); 
          First_GPS = 0;
        }
        GPS_to_ENU_Update(gps.location.lng(), gps.location.lat() , BMP_ALT);   // Needs Logitude and latitude in degrees - altitude in meters
        observation(0)=GPS_enu.east;    //GPS_ENU position in East axis (meters)
        observation(1)=GPS_enu.north;   //GPS_ENU position in North axis (meters)
        observation(2)=GPS_enu.up;      //GPS_ENU position in Up axis (meters)
        GPS_Time = millis();
      }
    ////////////////////////GPS+ALT////////////////////
    //////////////////////////IMU//////////////////////
      if (mpu.update())
      {
        if (millis() - IMU_Time > 10) 
        {
          Calibrate_IMU_RawData();

          IMU_to_ENU_Update(Calibrated.AccX, Calibrated.AccY, Calibrated.AccZ, 
                            mpu.getRoll()  , mpu.getPitch() , mpu.getYaw()  );

          observation(3)=mpu.getRoll();     // Roll
          observation(4)=mpu.getPitch();    // Pitch
          observation(5)=mpu.getYaw();      // Yaw
          Command(0)=IMU_enu.east ;         // Acceleration in East axis direction  (meter/second^2)
          Command(1)=IMU_enu.north;         // Acceleration in North axis direction (meter/second^2)
          Command(2)=IMU_enu.up   ;         // Acceleration in Up axis direction    (meter/second^2)

          Kalman_Update();
          ENU_to_GPS_Update (State(0), State(1), State(2));  // Convert output back to Geodetic Frame
          print_Everything();

          #ifdef SD_Card
          SD_Push();
          #endif

          IMU_Time = millis();
        }
      }
    //////////////////////////IMU//////////////////////
  #endif
  if (millis() - LED_Time > Blink_Interval)
  {
    if (gps.location.lat() > 0.0)
    {
      LED_Green_State = !LED_Green_State;
      LED_Red_State = HIGH;
      digitalWrite(LED_Green , LED_Green_State );
      digitalWrite(LED_Red   , LED_Red_State );
      LED_Time = millis();
    }
    else
    {
      LED_Green_State = HIGH;
      LED_Red_State = !LED_Red_State;
      digitalWrite(LED_Green , LED_Green_State );
      digitalWrite(LED_Red   , LED_Red_State );
      LED_Time = millis();
    }
  }
}
///////////////////////////////////////////////////////
void serialEvent1()                    // GPS interrupt
{
  while (Serial1.available())
      gps.encode(Serial1.read());

  if (gps.location.isValid())
  {
    BMP_ALT=bmp.readAltitude(); 
  }    
}
///////////////////////////////////////////////////////