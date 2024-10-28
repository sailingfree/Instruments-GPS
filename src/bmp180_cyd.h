//

#include <BMP180I2C.h>

#define BMP_I2C_ADDRESS 0x77

void setup_bmp180();
float get_temperature();
float get_pressure();
