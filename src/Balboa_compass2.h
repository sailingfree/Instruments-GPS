#include <vector.h>

void setupCompass();
// Returns a heading (in degrees) given an acceleration vector a due to gravity, a magnetic vector m, and a facing vector p (global).
int get_heading(const vector * a, const vector * m);

void printHeading();

void calibrate();