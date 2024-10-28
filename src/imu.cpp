// From here https://github.com/RCmags/imuFilter/tree/main

#include <basicMPU6050.h>
/*
 This sketch shows to integrate acceleration to obtain an estimate of velocity
*/

/* NOTE: 
   1. The accelerometer MUST be calibrated to integrate its output. Adjust the 
   AX, AY, AND AZ offsets until the sensor reads (0,0,1) at rest.

   2. REQUIRED UNITS:
      I. The gyroscope must output: radians/second
      II. The accelerometer output: g-force         [outputs 1 when sensor is static]
*/

//#include <basicMPU6050.h>   // Library for IMU sensor. See this link: https://github.com/RCmags/basicMPU6050
#include <accIntegral.h>
#include <imuFilter.h>
basicMPU6050<> imu;

#include <LSM303.h>
#include <L3G.h>

#include <10dof.h>

// LSM303 compass and accel
extern LSM303 compass;

// Gyro
extern L3G gyro;

// Seems to be a Gyro Device Type 0 4200D

// ========== IMU sensor ==========

// Gyro settings:
#define         LP_FILTER   3           // Low pass filter.                    Value from 0 to 6
#define         GYRO_SENS   0           // Gyro sensitivity.                   Value from 0 to 3
#define         ACCEL_SENS  0           // Accelerometer sensitivity.          Value from 0 to 3
#define         ADDRESS_A0  LOW         // I2C address from state of A0 pin.   A0 -> GND : ADDRESS_A0 = LOW
                                        //                                     A0 -> 5v  : ADDRESS_A0 = HIGH
// Accelerometer offset:
constexpr int   AX_OFFSET = 0;       // Use these values to calibrate the accelerometer. The sensor should output 1.0g if held level. 
constexpr int   AY_OFFSET = 0;       // These values are unlikely to be zero.
constexpr int   AZ_OFFSET = 0;

//-- Set template parameters:
/*
basicMPU6050<LP_FILTER,  GYRO_SENS,  ACCEL_SENS, ADDRESS_A0,
             AX_OFFSET,  AY_OFFSET,  AZ_OFFSET
            >imu;
*/
// =========== Settings ===========
accIntegral fusion;

// Filter coefficients                       //  Unit           
constexpr float GRAVITY = 9.81e3;            //  mm/s^2             Magnitude of gravity at rest. Determines units of velocity. [UNITS MUST MATCH ACCELERATION]
constexpr float SD_ACC  = 10000 / GRAVITY;    //  mm/s^2 / g-force   Standard deviation of acceleration. Deviations from zero are suppressed.
constexpr float SD_VEL  = 2000  / GRAVITY;    //  mm/s   / g-force   Standard deviation of velocity. Deviations from target value are suppressed.
constexpr float ALPHA   = 0.5;               //                     Gain of heading update - See example "output" for more information.

void imu_setup() {
//  Serial.begin(38400);
//  delay(1000);

  // calibrate IMU sensor
//  imu.setup();
//  imu.setBias();

  // initialize sensor fusion
  //fusion.setup( imu.ax(), imu.ay(), imu.az() );   // ALWAYS set initial heading with acceleration 
  fusion.setup();
  
  fusion.reset();  /* Use this function to zero velocity estimate */                               
}

void run_imu() { 

#if 0
     // Read the compass this also reads the accelerometer
    compass.read();

    int heading;

    float ax, ay, az;

    ax = compass.a.x;
    ay = compass.a.y;
    az = compass.a.z;

  //  ax *= 1000;
  //  ay *= 1000;
  //  az *= 1000;

    float mx, my, mz;
    mx = compass.m.x;
    my = compass.m.y;
    mz = compass.m.z;

    // Read the gyro;
    gyro.read();

    float gx, gy, gz;
    
    gx = gyro.g.x / 57;
    gy = gyro.g.y / 57;
    gz = gyro.g.z / 57;

  //  gx = gy = gz = 0;
  //  ax = ay = az = 0;
  //  az = GRAVITY;

    float axNorm, ayNorm;

    // Remap to between -1.0 and 1.0 and remove bias

  extern  float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);

//  ax = 0;
//  ay = 0;
//  az = 1;
// 10/4/23 run calibrate
    // min: min: {-16592, -15712, -19888}    max: {+16048, +16688, +14256}
 ax = mapfloat(ax, -16592, 16048, -1, 1);
 ay = mapfloat(ay, -15712, 16688, -1, 1);
 az = mapfloat(az, -19888, 14256, -1, 1);
 /*
 min and max after running calibration
 compass.a_min = (LSM303::vector<int16_t>){-15592, -15952, -19264};
    compass.a_max = (LSM303::vector<int16_t>){+15616, +16480, +13664};*/

  // Measure state:  
  
  vec3_t accel = { ax, ay, az };    // g-unit
  vec3_t gyro = { gx, gy, gz };     // radians/second
  Serial.printf("IMU ax %f ay %f az %f gx %f gy %f gz %f\n",
    ax,ay,az,gx,gy,gz);

  // Update heading and velocity estimate:
  
      // known measured velocity (target state). Estimate will be forced towards this vector
  vec3_t vel_t = {0,0,0};

  vel_t /= GRAVITY;                         // must have unit: g-force * second
  
      /* note: all coefficients are optional and have default values */
  fusion.update( gyro, accel, vel_t, SD_ACC, SD_VEL, ALPHA ); 
#endif

  LSM303::vector<float> g = gyroNorm();
  LSM303::vector<float> a = accelNorm();

  fusion.update(g.x, g.y, g.z, a.x, a.y, a.z, 0,0,0,SD_ACC, SD_VEL, ALPHA);

      // obtain velocity estimate
  vec3_t vel = fusion.getVel() * GRAVITY;   // scale by gravity for desired units
  
  // Display velocity components: [view with serial plotter]
//  Serial.print( vel.x, 2 );
//  Serial.print( " " );
//  Serial.print( vel.y, 2 );
//  Serial.print( " " );
//  Serial.print( vel.z, 2 );  
//  Serial.println();
}