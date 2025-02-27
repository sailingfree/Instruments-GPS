// Support for the adafruit 10dof board.
// https://github.com/adafruit/Adafruit_L3GD20_U

#include <L3G.h>
#include <LSM303.h>
#include <cyd_pins.h>
#include <StringStream.h>

void setup_10dof();

void printGyrodata(StringStream & output);
void printCompass(StringStream & output);
void printAccel(StringStream & output);

void lsm303_calibrate_m();
void lsm303_calibrate_a();
void printCompass2(StringStream & output);
void printCompass3();
void calibrate();
void run_fusion();
double mapfloat(double x, double in_min, double in_max, double out_min, double out_max);
LSM303::vector<double> mapvector(LSM303::vector<double> & x, 
                                    LSM303::vector<int16_t> & in_min, 
                                    LSM303::vector<int16_t> & in_max, 
                                    LSM303::vector<double> & out_min, 
                                    LSM303::vector<double> & out_max);
// Returns a heading (in degrees) given an acceleration vector a due to gravity, a magnetic vector m, and a facing vector p.
int get_heading(float acc[3], float mag[3], float p[3]);