#pragma once 

#include <Arduino.h>
#include <math.h>


/// DEFINES ///
// From running the lsm303_calibrate calibration I get these values
// min: {  -474,   -703,   -509}    max: {  +659,   +441,   +463}    // May 17 conservatory with GPS antenna
// min: {  -371,   -811,   -751}    max: {  +705,   +313,   +222}    // May 15 conservatory no GPS antenna

#define M_X_MIN -474 //-1000
#define M_Y_MIN -704 //-1000
#define M_Z_MIN -509 //-1000
#define M_X_MAX +659 //+1000
#define M_Y_MAX +441 //+1000
#define M_Z_MAX +463 //+1000

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

#define MtoNM(x) ((x) * 0.00054)      // metres to natcial miles

// gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain

#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second