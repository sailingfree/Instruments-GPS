
// last update 3/4/2018
/*
   magnetic compass program for Balboa by S. James Remington
   This program assumes that a LIS3MDL Balboa is horizontal, with X pointing toward 
   magnetic North for zero degrees heading
*/
#include <Wire.h>
#include <LIS3MDL.h>

typedef struct vector
{
  float x, y, z;
} vector;

LIS3MDL compass;

// Code initialization statements from magneto program, compass correction bias and rotation matrix
// scaled and rotated vectors will have norm ~ 1000 based on the example data (mag3_raw.csv)

float B[3] = { 9955.15, -7948.26, 8511.80};

float Ainv[3][3] = {{  0.25060,  0.02942, -0.02955},
  {  0.02942,  0.31692,  0.00789},
  { -0.02955,  0.00789,  0.30592}
};

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  if (!compass.init())
  {
    Serial.println("Failed to detect and initialize magnetometer!");
    while (1);
  }
  compass.enableDefault();
}

// Returns a set of distortion-corrected magnetic readings from the LIS3MDL

void read_data(vector * m)
{
  static float x, y, z;
  compass.read();
  x = (float) compass.m.x - B[0];
  y = (float) compass.m.y - B[1];
  z = (float) compass.m.z - B[2];
  m->x = Ainv[0][0] * x + Ainv[0][1] * y + Ainv[0][2] * z;
  m->y = Ainv[1][0] * x + Ainv[1][1] * y + Ainv[1][2] * z;
  m->z = Ainv[2][0] * x + Ainv[2][1] * y + Ainv[2][2] * z;
}

void loop()
{
  vector m;
  int heading;
  read_data(&m);
  
//   for the curious...
  Serial.print(m.x);
  Serial.print(", ");
  Serial.print(m.y);
  Serial.print(", ");
  Serial.print(m.z);
  Serial.println(",");

// assumes Balboa horizontal

  Serial.print("Heading: ");
  heading = 180.*atan2(m.y, m.x) / PI;
  if (heading < 0) heading = heading + 360;
  Serial.println(heading);
  delay(500);
}
