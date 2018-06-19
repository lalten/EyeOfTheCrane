#include <Wire.h>
#include <SPI.h>
#include <MadgwickAHRS.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5

Madgwick filter;

float rate = 1; // in kHz
float f = rate * 3.141593 / 2.0;

unsigned long microsPerReading, microsNext;
float accelScale, gyroScale;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
  }
  Serial.println("LSM9DS1 data read demo");
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS1 9DOF");
  // helper to just set the default scaling we want, see above!
  setupSensor();

  
  microsPerReading = 1000.0 / rate;
  microsNext = micros();

  //set the filter frequency to get proper values
  filter.begin(f);
}

void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}

void loop() {
      
  // check if it's time to read data and update the filter
  if (micros() >= microsNext) {
  microsNext = microsNext + microsPerReading;
  
  lsm.read();  // read the data from the sensor to get a new event 
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);

  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float ax_val, ay_val, az_val;
  float gx_val, gy_val, gz_val;
  float mx_val, my_val, mz_val;
  float roll, pitch, yaw;

  ax = a.acceleration.x;
  ay = a.acceleration.y;
  az = a.acceleration.z;
  gx = g.gyro.x;
  gy = g.gyro.y;
  gz = g.gyro.z;
  mx = m.magnetic.x;
  my = m.magnetic.y;
  mz = m.magnetic.z;

  // scale values if necessary (depends on sensor output)
  ax_val = convertRawAcc(ax);
  ay_val = convertRawAcc(ay);
  az_val = convertRawAcc(az);
  gx_val = convertRawGyro(gx);
  gy_val = convertRawGyro(gy);
  gz_val = convertRawGyro(gz);
  mx_val = mx;
  my_val = my;
  mz_val = mz;
  
  
  // update the filter, which computes orientation
  filter.updateIMU(gx_val, gy_val, gz_val, ax_val, ay_val, az_val);   // w/o magnetic field
  //  filter.update(gx_val, gy_val, gz_val, ax_val, ay_val, az_val, mx_val, my_val, mz_val);    // w/ magnetic field
    
  roll = filter.getRoll();
  pitch = filter.getPitch();
  yaw = filter.getYaw();


  // print the yaw, pitch and roll
  Serial.print("Orientation: ");
  Serial.print(yaw);
  Serial.print(" ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.println(roll);

  // print raw data
  //  Serial.print("Accel X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2");
  //  Serial.print("\tY: "); Serial.print(a.acceleration.y);     Serial.print(" m/s^2 ");
  //  Serial.print("\tZ: "); Serial.print(a.acceleration.z);     Serial.println(" m/s^2 ");
  //
  //  Serial.print("Mag X: "); Serial.print(m.magnetic.x);   Serial.print(" gauss");
  //  Serial.print("\tY: "); Serial.print(m.magnetic.y);     Serial.print(" gauss");
  //  Serial.print("\tZ: "); Serial.print(m.magnetic.z);     Serial.println(" gauss");
  //
  //  Serial.print("Gyro X: "); Serial.print(g.gyro.x);   Serial.print(" dps");
  //  Serial.print("\tY: "); Serial.print(g.gyro.y);      Serial.print(" dps");
  //  Serial.print("\tZ: "); Serial.print(g.gyro.z);      Serial.println(" dps");
  //
  //  Serial.println();

  }
}

float convertRawGyro(float gRaw) {
  // must be in rad/s
  // 1 deg/s = 0.0174533 rad/s
  float scale = 0.0174533;
  float g = (gRaw * scale);
  return g;
}

float convertRawAcc(float aRaw) {
  // must be in m/s^2
  // 1 g = 9.80665 m/s^2
  float scale = 1.0;
  float a = (aRaw * scale);
  return a;
}

