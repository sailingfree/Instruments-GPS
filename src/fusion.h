// Use the Arduino sensorfusion library

#include <SensorFusion.h>

void fusion_setup();
void fusion_work(double gx, double gy, double gz,
                double ax, double ay, double az,
                double mx, double my, double mz);
