// Support for the adafruit 10dof board.
// https://github.com/adafruit/Adafruit_L3GD20_U

#include <L3G.h>
#include <LSM303.h>
#include <cyd_pins.h>
#include <StringStream.h>
#include <imu.h>

typedef struct {
    float x, y, z;
} vect;

void setup_10dof();

void printGyrodata(StringStream& output);
void printCompass(StringStream& output);
void printAccel(StringStream& output);

void lsm303_calibrate_m();
void lsm303_calibrate_a();
void printCompass2(StringStream& output);
void printCompass3();
void calibrate2();
void run_fusion();
void setGyroBias();

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
LSM303::vector<float> mapvector(LSM303::vector<float>& x,
                                 LSM303::vector<int16_t>& in_min,
                                 LSM303::vector<int16_t>& in_max,
                                 LSM303::vector<float>& out_min,
                                 LSM303::vector<float>& out_max);
// Returns a heading (in degrees) given an acceleration vector a due to gravity, a magnetic vector m, and a facing vector p.
int get_heading(float acc[3], float mag[3], float p[3]);

LSM303::vector<float> gyroRaw();
LSM303::vector<float> compassRaw();
LSM303::vector<float> accelRaw();

LSM303::vector<float> gyroNorm();
LSM303::vector<float> compassNorm();
LSM303::vector<float> accelNorm();
LSM303::vector<float> accelMss();


