#include <Wire.h>
#include <LIS3MDL.h>
// mods by S. James Remington 3/2018

LIS3MDL mag;
LIS3MDL::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = { -32768, -32768, -32768};

char report[80];
LIS3MDL::vector<int16_t> m_off, m_scl;

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  
  // enter keyboard character and hit "send" to start
  while (!Serial.available()); //wait for keyboard entry

  if (!mag.init())
  {
    Serial.println("Failed to detect and initialize magnetometer!");
    while (1);
  }

  mag.enableDefault();
  unsigned long start = millis();

  // calibration loop. Enter a keyboard character to start, then rotate Balboa in 3D, exploring all orientations
  // loop until no change in max/min withing 10 seconds

  Serial.println("Collecting calibration data");

  while (millis() - start < 10000UL) {
    delay(100);
    mag.read();
    snprintf(report, sizeof(report), "%+6d,%+6d,%+6d",
             mag.m.x, mag.m.y, mag.m.z);
    Serial.println(report);
    if (mag.m.x < running_min.x) {
      running_min.x = mag.m.x;
      start = millis();
    }
    if (mag.m.x > running_max.x) {
      running_max.x = mag.m.x;
      start = millis();
    }
    if (mag.m.y < running_min.y) {
      running_min.y = mag.m.y;
      start = millis();
    }
    if (mag.m.y > running_max.y) {
      running_max.y = mag.m.y;
      start = millis();
    }
    if (mag.m.z < running_min.z) {
      running_min.z = mag.m.z;
      start = millis();
    }
    if (mag.m.z > running_max.z) {
      running_max.z = mag.m.z;
      start = millis();
    }
  }

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
  delay(10000); //show for 10 seconds
}

// print out some centered and scaled magnetometer values, while user rotates Balboa in space.
// This simple scaling approach does not work well due to severe hard iron distortion

void loop()
{
  float m[3];
  mag.read();
  m[0] = ((float) mag.m.x - m_off.x) / m_scl.x;
  m[1] = ((float) mag.m.y - m_off.y) / m_scl.y;
  m[2] = ((float) mag.m.z - m_off.z) / m_scl.z;
  for (int i = 0; i < 3; i++) {
    Serial.print(m[i]);
    Serial.print(",");
  }
  Serial.println();
  delay(200);
}
