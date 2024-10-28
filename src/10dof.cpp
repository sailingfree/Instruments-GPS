#include <10dof.h>
#include <LSM303.h>
#include <Wire.h>
#include <magneto.h>
#include <imu.h>
#include <GwPrefs.h>
#include <defines.h>

//#include "ekfNavINS.h"

//#include <fusion.h>

// Gyro
L3G gyro;
// acceleration due to gravity
static constexpr float G = 9.807f;

typedef L3G::deviceType l3gType;

// LSM303 compass and accel
LSM303 compass;
LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};

typedef LSM303::deviceType compassType;




///////////////////////////////////////////////////////////////////
// Setup functions
///////////////////////////////////////////////////////////////////
void setup_compass() {
    compassType type;
    String type_str = "Unknown";

    compass.init();

    compass.enableDefault();

    type = compass.getDeviceType();

    switch(type) {
         case LSM303::deviceType::device_DLH:
            type_str = "device_DLH";
            break;
        case LSM303::deviceType::device_DLM:
            type_str = "device_DLM";
            break;
        case LSM303::deviceType::device_DLHC:
            type_str = "device_DLHC";
            break;
        case LSM303::deviceType::device_D:
            type_str = "device_DLHC";
            break;
        case LSM303::deviceType::device_auto:
            type_str = "device_auto";
            break;
        default:
            type_str = "Unknown type";
            break;
    }

    Serial.printf("Compass type %d %s\n", type, type_str.c_str());
    
    /*
    Calibration values; the default values of +/-32767 for each axis
    lead to an assumed magnetometer bias of 0. Use the Calibrate example
    program to determine appropriate values for your particular unit.
    */
    compass.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
    compass.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};

 

int16_t min_m_x, min_m_y, min_m_z, max_m_x, max_m_y, max_m_z;

int16_t iscal = GwGetValInt(IS_CAL);

// If not initialised then set some values from a static calibration.
if(!iscal) {
    GwSetValInt(MIN_M_X, -371);
    GwSetValInt(MIN_M_Y, -811);
    GwSetValInt(MIN_M_Z, -751);
    GwSetValInt(MAX_M_X, +705);
    GwSetValInt(MAX_M_Y, +313);
    GwSetValInt(MAX_M_Z, +222);
    GwSetValInt(IS_CAL, 1);
}

min_m_x = GwGetValInt(MIN_M_X);
min_m_y = GwGetValInt(MIN_M_Y);
min_m_z = GwGetValInt(MIN_M_Z);
max_m_x = GwGetValInt(MAX_M_X);
max_m_y = GwGetValInt(MAX_M_Y);
max_m_z = GwGetValInt(MAX_M_Z);


    compass.m_min = (LSM303::vector<int16_t>){min_m_x, min_m_y, min_m_z};
    compass.m_max = (LSM303::vector<int16_t>){max_m_x, max_m_y, max_m_z};

    // Similar for accelerometer
    // min: {-17392, -15952, -19264}    max: {+15592, +16480, +13664}

    compass.a_min = (LSM303::vector<int16_t>){-15592, -15952, -19264};
    compass.a_max = (LSM303::vector<int16_t>){+15616, +16480, +13664};

    // 10/4/23 run calibrate
    // min: min: {-16592, -15712, -19888}    max: {+16048, +16688, +14256}
    // min: {-16560, -16160, -20496}    max: {+15840, +16800, +13904}    // May 17 conservatory
    compass.a_min = (LSM303::vector<int16_t>){-16560, -16160, -20496};
    compass.a_max = (LSM303::vector<int16_t>){+15840, +16800, +13904};
}

void setup_10dof() {
    l3gType type;
    String type_str = "Unknown type";

    if (!gyro.init()) {
        Serial.println("Failed to autodetect gyro type!");
        while (1)
            ;
    }

    gyro.enableDefault();
    type = gyro.getDeviceType();
    switch (type) {
        case L3G::device_4200D:
            type_str = "4200D";
            break; 
        case L3G::device_D20: 
            type_str = "D20";
            break;
        case L3G::device_D20H:
            type_str = "D20H";
            break;
        case L3G::device_auto:
            type_str = "AUTO";
            break;
        default :
            type_str = "Unknown";
            break;
    }

    Serial.printf("Gyro Device Type %d %s\n", type, type_str.c_str());

    setup_compass();
}

///////////////////////////////////////////////////////////////////
// Print raw sensor data
///////////////////////////////////////////////////////////////////
void printGyrodata(StringStream& output) {

    LSM303::vector<float> g = gyroNorm();

    Serial.printf(">gyx:%f\n", g.x);
    Serial.printf(">gyy:%f\n", g.x);
    Serial.printf(">gyz:%f\n", g.z);
}

void printCompass(StringStream& output) {
    compass.read();

    /*
    When given no arguments, the heading() function returns the angular
    difference in the horizontal plane between a default vector and
    north, in degrees.

    The default vector is chosen by the library to point along the
    surface of the PCB, in the direction of the top of the text on the
    silkscreen. This is the +X axis on the Pololu LSM303D carrier and
    the -Y axis on the Pololu LSM303DLHC, LSM303DLM, and LSM303DLH
    carriers.

    To use a different vector as a reference, use the version of heading()
    that takes a vector argument; for example, use

      compass.heading((LSM303::vector<int>){0, 0, 1});

    to use the +Z axis as a reference.
    */
    float heading = compass.heading();

    output.printf("Mag: X:%d Y:%d Z:%d HDG %.3f\n",
                  compass.m.x, compass.m.y, compass.m.z, heading);

   // Serial.print(output.data);

  //  Serial.printf(">compx:%d\n", compass.m.x);
  //  Serial.printf(">compy:%d\n", compass.m.y);
  //  Serial.printf(">compz:%d\n", compass.m.z);
  //  Serial.printf(">heading:%.3f\n", heading);
}

// Print the accelerometer data
void printAccel(StringStream& output) {

    LSM303::vector<float> vals = accelNorm();

    //Serial.printf(">accx:%.3f\n", vals.x);
    //Serial.printf(">accy:%.3f\n", vals.y);
    //Serial.printf(">accz:%.3f\n", vals.z);

 output.printf("Accel: X:%.3f Y:%.3f Z:%.3f\n",
                 vals.x, vals.y, vals.z);
}

static char report[80];

// Run the compass calibration to get max and min values in all 3 axis
void lsm303_calibrate_m() {
    compass.read();

    running_min.x = min(running_min.x, compass.m.x);
    running_min.y = min(running_min.y, compass.m.y);
    running_min.z = min(running_min.z, compass.m.z);

    running_max.x = max(running_max.x, compass.m.x);
    running_max.y = max(running_max.y, compass.m.y);
    running_max.z = max(running_max.z, compass.m.z);

    snprintf(report, sizeof(report), "min: {%+6d, %+6d, %+6d}    max: {%+6d, %+6d, %+6d}",
             running_min.x, running_min.y, running_min.z,
             running_max.x, running_max.y, running_max.z);
    Serial.println(report);

    delay(100);
}

// Run the accelerometer calibration to get max and min values
// in all 3 axis
void lsm303_calibrate_a() {
    compass.read();

    running_min.x = min(running_min.x, compass.a.x);
    running_min.y = min(running_min.y, compass.a.y);
    running_min.z = min(running_min.z, compass.a.z);

    running_max.x = max(running_max.x, compass.a.x);
    running_max.y = max(running_max.y, compass.a.y);
    running_max.z = max(running_max.z, compass.a.z);

    snprintf(report, sizeof(report), "min: {%+6d, %+6d, %+6d}    max: {%+6d, %+6d, %+6d}",
             running_min.x, running_min.y, running_min.z,
             running_max.x, running_max.y, running_max.z);
  Serial.println(report);

   delay(100);
}

// Heading code from here https://forum.pololu.com/t/correcting-the-balboa-magnetometer/14315
// https://forum.pololu.com/uploads/default/original/2X/f/fde22af8f432eaecdd0b900c74b3596aeb7c8c74.zip

// Code initialization statements from magneto program, compass correction bias and rotation matrix
// scaled and rotated vectors will have norm ~ 1000 based on the example data (mag3_raw.csv)

typedef struct vector {
    float x, y, z;
} vector;

// Calib data from running the calib2 and using magneto

float B[3] = {-86.47, -151.97, -13.18};
//
float Ainv[3][3] = {{2.01731, 0.03164, 0.16148},
                    {0.03164, 2.03701, -0.07833},
                    {0.16148, -0.07833, 2.26656}};

// float B[3] {  171.73, -117.18,  -22.19};

// float Ainv[3][3] {{  0.88229, -0.04538,  0.05431},
//              { -0.04538,  0.99437, -0.02458},
//              {  0.05431, -0.02458,  1.14117}};

// Calibrate data jan 15 2024 21:35
// float B[3] {  601.13,  183.85, -212.91};

// float Ainv[3][3] {{  0.76115, -0.12926,  0.10827},
//              { -0.12926,  0.78797,  0.00392},
//              {  0.10827,  0.00392,  0.95147}};

// Returns a set of distortion-corrected magnetic readings from the LIS3MDL
void read_data(vector* m) {
    static float x, y, z;
    compass.read();
    x = (float)compass.m.x - B[0];
    y = (float)compass.m.y - B[1];
    z = (float)compass.m.z - B[2];
    m->x = Ainv[0][0] * x + Ainv[0][1] * y + Ainv[0][2] * z;
    m->y = Ainv[1][0] * x + Ainv[1][1] * y + Ainv[1][2] * z;
    m->z = Ainv[2][0] * x + Ainv[2][1] * y + Ainv[2][2] * z;
}

void printCompass2(StringStream& output) {
    vector m;
    int heading;
    read_data(&m);

    //   for the curious...
    //Serial.print(m.x);
    //Serial.print(", ");
    //Serial.print(m.y);
    //Serial.print(", ");
    //Serial.print(m.z);
    //Serial.println(",");

  //  Serial.printf(">mx:%f\n", m.x);
  //  Serial.printf(">my:%f\n", m.y);
  //  Serial.printf(">mz:%f\n", m.z);

    // assumes compass horizontal

    heading = 180. * atan2(m.y, m.x) / PI;
    if (heading < 0) {
        heading = heading + 360;
    }
    output.println(heading);
    //Serial.print(output.data);

    int16_t ax = compass.a.x >> 4;
    int16_t ay = compass.a.y >> 4;
    int16_t az = compass.a.z >> 4;

    float axNorm, ayNorm;
    //--- Pitch and Roll calculation
    axNorm = ax / sqrt(ax * ax + ay * ay + az * az);
    ayNorm = ay / sqrt(ax * ax + ay * ay + az * az);

    float pitch = asin(-axNorm);
    float roll = asin(ayNorm / cos(pitch));

    //Serial.printf("Pitch%f Roll%f ", pitch * 57.2958, roll * 57.2958);

    // Get the compass and apply tilt compensation - from here: https://forum.pololu.com/t/lsm303d-tilt-formula/15052
    //--- Final data

    float magXheading = m.x * cos(pitch) + m.z * sin(pitch);
    float magYheading = m.x * sin(roll) * sin(pitch) + m.y * cos(roll) - m.z * sin(roll) * cos(pitch);

    int compHeading;

    compHeading = 180. * atan2(magYheading, magXheading) / PI;
    float declination = 0.00465421;  // Bournemouth East 0degrees 16 minutes;
    compHeading += declination * 180 / PI;
    if (compHeading < 0) {
        compHeading = compHeading + 360;
    }

    //Serial.printf("Compensated heading %d MX %f MY %f\n", compHeading, magXheading, magYheading);
}

// Calibration from the balboa code
LSM303::vector<int16_t> m_off, m_scl;
void calibrate2() {
    // enter keyboard character and hit "send" to start
    while (!Serial.available())
        ;  // wait for keyboard entry

    if (!compass.init()) {
        Serial.println("Failed to detect and initialize magnetometer!");
        while (1)
            ;
    }

    compass.enableDefault();
    unsigned long start = millis();

    // calibration loop. Enter a keyboard character to start, then rotate Balboa in 3D, exploring all orientations
    // loop until no change in max/min withing 10 seconds

    Serial.println("// Collecting calibration data");

    int16_t rows = 256;
#define COLS 3

    int16_t(*data)[COLS];
    size_t bytes = sizeof(int16_t) * rows * 3;
    data = (int16_t(*)[COLS])malloc(bytes);
    if (!data) {
        Serial.println("Failed to allocate data array\n");
        return;
    } else {
        Serial.printf("Data array is %d bytes at address 0x%p\n", bytes, data);
    }
    int16_t current = 0;

    while (current < rows) {
        delay(100);
        int16_t oldx, oldy, oldz;
        int16_t x, y, z;
        compass.read();

        x = compass.m.x;
        y = compass.m.y;
        z = compass.m.z;
    running_min.x = min(running_min.x, compass.m.x);
    running_min.y = min(running_min.y, compass.m.y);
    running_min.z = min(running_min.z, compass.m.z);

    running_max.x = max(running_max.x, compass.m.x);
    running_max.y = max(running_max.y, compass.m.y);
    running_max.z = max(running_max.z, compass.m.z);
    
        if (x != oldx || y != oldy || z != oldz) {
            // had a change so use it
            snprintf(report, sizeof(report), "{%+6d,%+6d,%+6d},",
                     x, y, z);

            Serial.println(report);

            data[current][0] = x;
            data[current][1] = y;
            data[current][2] = z;
            oldx = x;
            oldy = y;
            oldz = z;
            current++;
        }
    }
    Serial.println("};");
    Serial.println("Done collecting calibration data");
    snprintf(report, sizeof(report), "min: {%+6d, %+6d, %+6d}   max: {%+6d, %+6d, %+6d}",
             running_min.x, running_min.y, running_min.z,
             running_max.x, running_max.y, running_max.z);
    Serial.println(report);
    m_off.x = (running_max.x + running_min.x) / 2;
    m_off.y = (running_max.y + running_min.y) / 2;
    m_off.z = (running_max.z + running_min.z) / 2;
    m_scl.x = (running_max.x - running_min.x) / 2;
    m_scl.y = (running_max.y - running_min.y) / 2;
    m_scl.z = (running_max.z - running_min.z) / 2;
    snprintf(report, sizeof(report), "off: {%+6d, %+6d, %+6d}   scl: {%+6d, %+6d, %+6d}",
             m_off.x, m_off.y, m_off.z,
             m_scl.x, m_scl.y, m_scl.z);
    Serial.println(report);

    // Process the captured data in magneto to calculate the offsets
    magneto(data, rows);
}

// Version using vectors from here https://github.com/jremington/LSM9DS1-AHRS/blob/main/LSM9DS1_tiltcomp/LSM9DS1_tiltcomp.ino
// basic vector operations
void vector_cross(LSM303::vector<float> a, LSM303::vector<float>b, LSM303::vector<float> & out) {
    out.x = a.y * b.z - a.z * b.y;
    out.y = a.z * b.x - a.x* b.z;
    out.z = a.x * b.y - a.y * b.x;
}

float vector_dot(LSM303::vector<float> a, LSM303::vector<float> b) {
    float result;

    result = a.x * b.x + a.y * b.y + a.z * b.z;
    return result;
 //   a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void vector_normalize(LSM303::vector<float> & a) {
    float mag = sqrt(vector_dot(a, a));
    a.x /= mag;
    a.y /= mag;
    a.z /= mag;
}



// Returns a heading (in degrees) given an acceleration vector a due to gravity, a magnetic vector m, and a facing vector p.
int get_heading(LSM303::vector<float> acc, LSM303::vector<float> mag, LSM303::vector<float> p) {
    LSM303::vector<float> W, N;  // derived direction vectors


    // cross "Up" (acceleration vector, g) with magnetic vector (magnetic north + inclination) with  to produce "West"
    vector_cross(acc, mag, W);
    vector_normalize(W);

    // cross "West" with "Up" to produce "North" (parallel to the ground)
    vector_cross(W, acc, N);
    vector_normalize(N);

    // compute heading in horizontal plane, correct for local magnetic declination
    float declination = 0.00465421;  // Bournemouth East 0degrees 16 minutes;

    int heading = round(atan2(vector_dot(W, p), vector_dot(N, p)) * 180 / M_PI + declination);
    heading = -heading;               // conventional nav, heading increases North to East
    heading = (heading + 720) % 360;  // apply compass wrap
    return heading;
}

void printCompass3() {
    vector m;
    int heading;    
    
    // facing vector Y marking on sensor board points toward yaw = 0
    LSM303::vector<float> p;
    p.x = 0;
    p.y = 1; 
    p.z = 0;
 //   read_data(&m);

    // Convert to array
 //   float mag[3] = {m.x, m.y, m.z};

    // Get the accel data
 //   float accel[3] = {compass.a.x, compass.a.y, compass.a.z};

    LSM303::vector<float> mag = compassNorm();
    LSM303::vector<float> accel = accelNorm();

    heading = get_heading(accel, mag, p);

    Serial.printf("Vector heading %d\n", heading);
}

void run_fusion() {
  #if 0

    // Read the compass this also reads the accelerometer
    compass.read();

    int heading;

    float ax, ay, az;

    ax = compass.a.x;
    ay = compass.a.y;
    az = compass.a.z;

    float mx, my, mz;
    mx = compass.m.x;
    my = compass.m.y;
    mz = compass.m.z;

    // Read the gyro;
    gyro.read();

    float gx, gy, gz;
    gx = gyro.g.x;
    gy = gyro.g.y;
    gz = gyro.g.z;

    fusion_work(gx, gy, gz, ax, ay, az, mx, my, mz);
    #endif
}

// Map a value to another.
// Removes bias
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
    float result = x;
    float temp = ((float)in_min + in_max) / 2;
    x -= temp;
    //  Serial.printf("TEMP %f MIN %f MAX %f OLD %f ADJ %f\n", temp, in_min, in_max, result, x);

    // Adjust the max and min by the bias
    float in_min_t = in_min - temp;
    float in_max_t = in_max - temp;
    result = (x - in_min_t) * (out_max - out_min) / (in_max_t - in_min_t) + out_min;
    return result;
}

// Map using vectors
LSM303::vector<float> mapvector(LSM303::vector<float>& invals, LSM303::vector<int16_t>& in_min, LSM303::vector<int16_t>& in_max, LSM303::vector<float>& out_min, LSM303::vector<float>& out_max) {
    LSM303::vector<float> result;
    result.x = mapfloat(invals.x, in_min.x, in_max.x, out_min.x, out_max.x);
    result.y = mapfloat(invals.y, in_min.y, in_max.y, out_min.y, out_max.y);
    result.z = mapfloat(invals.z, in_min.z, in_max.z, out_min.z, out_max.z);
    return result;
}


// Raw values from the sensors
LSM303::vector<float> gyroRaw() {
    LSM303::vector<float> raw;

    gyro.read();
    raw.x = gyro.g.x;
    raw.y = gyro.g.y;
    raw.z = gyro.g.z;

    return raw;
}

LSM303::vector<float> compassRaw(){
    LSM303::vector<float> raw;

    compass.read();
    raw.x = compass.m.x;
    raw.y = compass.m.y;
    raw.z = compass.m.z;

    return raw;
}

LSM303::vector<float> accelRaw(){
    LSM303::vector<float> raw;

    compass.read();
    raw.x = compass.a.x;
    raw.y = compass.a.y;
    raw.z = compass.a.z;

    return raw;
}

LSM303::vector<float> gyroBias = {0,0,0};

// Determine the static bias for the gyro. Done at startup while hopefully at rest
void setGyroBias() {
    static const uint16_t count = 1000;

    // Accumulate the readings
    for(int i = 0; i < count; i++) {
        LSM303::vector<float> g = gyroRaw();
        gyroBias.x += g.x;
        gyroBias.y += g.y;
        gyroBias.z += g.z;
    
        delayMicroseconds(100);
    }

    // calculate the means
    gyroBias.x /= count;
    gyroBias.z /= count;
    gyroBias.y /= count;
}

// Values after calibration conversions and offsets applied
// Gyro reading in Radians/sec
LSM303::vector<float> gyroNorm(){
    LSM303::vector<float> raw = gyroRaw();
    LSM303::vector<float> norm;

    norm.x = raw.x - gyroBias.x;
    norm.y = raw.y - gyroBias.y;
    norm.z = raw.z - gyroBias.z;

//Serial.printf("GX %f GY %f GZ %f\n", raw.x, raw.y, raw.z);
    norm.x *= DEG_TO_RAD;
    norm.y *= DEG_TO_RAD;
    norm.z *= DEG_TO_RAD;
//Serial.printf("NGX %f NGY %f NGZ %f\n", norm.x, norm.y, norm.z);

    return norm;
}

//
static const int16_t MAX_SCALE = 32767;
LSM303::vector<float> compassNorm(){
    LSM303::vector<float> norm;
    LSM303::vector<float> invals = compassRaw();
    LSM303::vector<float> out_min = {-MAX_SCALE, -MAX_SCALE, -MAX_SCALE};
    LSM303::vector<float> out_max = {MAX_SCALE, MAX_SCALE, MAX_SCALE};
    norm = mapvector(invals, compass.m_min, compass.m_max, out_min, out_max);
    return norm;
}

// Return the accelretaion as a vector where each axis is between -1 and +1 g
// Removes any bias found during calibration
LSM303::vector<float> accelNorm(){

    LSM303::vector<float> accel = accelRaw();
    LSM303::vector<float> newvals;

    // Remap to between -1.0 and 1.0 and remove bias
    LSM303::vector<float> minv = {-1.0, -1.0, -1.0}, maxv = {1.0, 1.0, 1.0};
    newvals = mapvector(accel, compass.a_min, compass.a_max, minv, maxv);

     return newvals;
}



// Return acceleration as m/s/s
LSM303::vector<float> accelMss(){

    LSM303::vector<float> accel = accelRaw();
    LSM303::vector<float> newvals;

    // Remap to between -1.0 and 1.0 and remove bias
    LSM303::vector<float> minv = {-G, -G, -G}, maxv = {G, G, G};
    newvals = mapvector(accel, compass.a_min, compass.a_max, minv, maxv);

    return newvals;
}