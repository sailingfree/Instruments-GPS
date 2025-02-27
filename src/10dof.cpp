#include <10dof.h>
#include <LSM303.h>
#include <Wire.h>

// Gyro
L3G gyro;

typedef L3G::deviceType l3gType;

// LSM303 compass and accel
LSM303 compass;
LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};

typedef LSM303::deviceType compassType;

///////////////////////////////////////////////////////////////////
// Setup functions
///////////////////////////////////////////////////////////////////
void setup_compass() {
    compass.init();
    compass.enableDefault();

    /*
    Calibration values; the default values of +/-32767 for each axis
    lead to an assumed magnetometer bias of 0. Use the Calibrate example
    program to determine appropriate values for your particular unit.
    */
    compass.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
    compass.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};

    // From running the lsm303_calibrate calibration I get these values
    //min: {  -441,   -825,   -455}    max: {  +441,   +241,   +545}  // feb 20 2025
    compass.m_min = (LSM303::vector<int16_t>){-441, -825, -455};
    compass.m_max = (LSM303::vector<int16_t>){+441, +241, +545};
}

void setup_10dof() {
    LSM303::deviceType type;
    String type_str = "Unknown type";
    setup_compass();

    type = compass.getDeviceType();
    switch (type) {
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
    Serial.printf("LSM303 Device Type %d %s\n", type, type_str.c_str());
}

///////////////////////////////////////////////////////////////////
// Print raw sensor data
///////////////////////////////////////////////////////////////////
void printGyrodata(StringStream& output) {
    gyro.read();

    output.printf("Gyro x: %d y: %d z: %d\n", gyro.g.x, gyro.g.y, gyro.g.z);
    Serial.print(output.data);

    Serial.printf(">gyx:%d\n", gyro.g.x);
    Serial.printf(">gyy:%d\n", gyro.g.y);
    Serial.printf(">gyz:%d\n", gyro.g.z);
}

void printCompass(StringStream& output) {
    static time_t last = 0;
    time_t now = millis();
    if (now > last + 1000) {
        last = now;

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
        float heading = compass.heading(LSM303::vector<int16_t>{0, 0, 1});
        //float heading = compass.heading();
        if (heading < 0.0) heading += 360.0;
        if (heading > 360.0) heading -= 360.0;

        // output.printf("Mag: X:%d Y:%d Z:%d HDG %.3f\n",
        // compass.m.x, compass.m.y, compass.m.z, heading);

        // Serial.print(output.data);

        // Serial.printf(">compx:%d\n", compass.m.x);
        // Serial.printf(">compy:%d\n", compass.m.y);
        // Serial.printf(">compz:%d\n", compass.m.z);
        // Serial.printf(">heading:%.3f\n", heading);
        Serial.printf("Mag: X:%d Y:%d Z:%d HDG %.3f Accel: X:%d Y:%d Z:%d                                              \r", compass.m.x, compass.m.y, compass.m.z, heading, compass.a.x, compass.a.y, compass.a.z);
    }
}

// Print the accelerometer data
void printAccel(StringStream& output) {
    compass.read();

    int16_t ax = compass.a.x;
    int16_t ay = compass.a.y;
    int16_t az = compass.a.z;

    float axNorm, ayNorm;

    // Remap to between -1.0 and 1.0 and remove bias
    //    LSM303::vector<double> maxv = {1.0, 1.0, 1.0}, minv = {-1.0, -1.0, -1.0}, vals = {ax, ay, az};
    //    vals = mapvector(vals, compass.a_min, compass.a_max, minv, maxv);

    //    Serial.printf(">accx:%.3f\n", vals.x);
    //    Serial.printf(">accy:%.3f\n", vals.y);
    //    Serial.printf(">accz:%.3f\n", vals.z);

    //    output.printf("Accel: X:%.3f Y:%.3f Z:%.3f\n",
    //                  vals.x, vals.y, vals.z);
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


void calibrate() {
  Serial.printf("Pleas start rotating the unit in all directions\n");
  time_t now = millis();

  while(millis() < now + 60000) {
    lsm303_calibrate_m();
  }
  Serial.printf("compass.m_min = (LSM303::vector<int16_t>){%+6d, %+6d, %+6d}\n", running_min.x, running_min.y, running_min.z);
  Serial.printf("compass.m_max = (LSM303::vector<int16_t>){%+6d, %+6d, %+6d}\n", running_max.x, running_max.y, running_max.z);

  Serial.printf("Done\n");
}