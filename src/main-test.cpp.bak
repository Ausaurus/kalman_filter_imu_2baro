#include "Arduino.h"
#include "ICM42688.h"
#include "MS5611.h"
#include "KalmanFilter.h"
#include "madgwickFilter.h"

float quat[4] = {1.0f,0.0f,0.0f,0.0f};
float beta = 0.6f;

ICM42688 IMU(Wire, 0x68);
MS5611 baroSensor(0x77);
KalmanFilter kalman;

#define Z_VARIANCE 300.0f
#define ZACCEL_VARIANCE 200.0f
#define ZACCELBIAS_VARIANCE 1.0f

void setup() {
  // serial to display data
  Serial.begin(115200);
  // while(!Serial) {}

  // start communication with IMU
  int status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }

  Wire.begin();
  if (baroSensor.begin() == true)
  {
    Serial.print("MS5611 found: ");
    Serial.println(baroSensor.getAddress());
  }
  else
  {
    Serial.println("MS5611 not found. halt.");
    //  while (1);
  }
  pinMode(39, OUTPUT);
  float total = 0;
  for (int i = 0; i < 4; i++)
  {
    baroSensor.read();
    total += baroSensor.getAltitude();
  }
  float averageCm = total / 4 * 100;
  kalman.Configure(Z_VARIANCE, ZACCEL_VARIANCE, ZACCELBIAS_VARIANCE, averageCm, 0.0f, 0.0f);
}

void loop()
{
  // 1. Enforce 100Hz timing to match madgwickFilter.h DELTA_T (0.01f)
  digitalWrite(LED_BUILTIN, (millis() / 250) % 2);
  static unsigned long lastMicros = 0;
  if (micros() - lastMicros < 10000) { // 10000 us = 10 ms = 100Hz
    return; 
  }
  float dt = (micros() - lastMicros) / 1000000.0f; // Calculate actual dt if you want to be precise, 
                                                   // though the filter uses the hardcoded macro.
  lastMicros = micros();

  // 2. Read Sensors
  baroSensor.read();
  IMU.getAGT();
  
  // 3. Get raw data
  float fax = IMU.accX();
  float fay = IMU.accY();
  float faz = IMU.accZ();
  
  // 4. Convert Gyro from Degrees/Sec to Radians/Sec
  float fgx = IMU.gyrX() * (PI / 180.0f);
  float fgy = IMU.gyrY() * (PI / 180.0f);
  float fgz = IMU.gyrZ() * (PI / 180.0f);
  
  if (abs(fgz) < 0.02f) { // 0.02 rads is approx 1 deg
    fgz = 0.0f; 
  }

  // 5. CRITICAL: Call the Madgwick Filter
  // Note: madgwickFilter.c expects normalized accel? 
  // Actually, line 43 of madgwickFilter.c calls quat_Normalization(&q_a), 
  // so passing raw G's is fine as long as they aren't all zero.
  imu_filter(fax, fay, faz, fgx, fgy, fgz);

  // 6. Get Angles
  float angleX = 0, angleY = 0, angleZ = 0;
  eulerAngles(q_est, &angleX, &angleY, &angleZ);

  // 7. (Optional) Update Kalman with Barometer data
  // You need to decide what 'z' and 'a' are. 
  // Usually z = baro altitude, a = vertical acceleration (world frame)
  // Converting body-frame accels (fax, fay, faz) to world-frame requires 
  // rotating them by q_est. This is complex to add here without more math functions.
  // Simple test for now:
  float estimatedZ, estimatedV;
  float baroAlt = baroSensor.getAltitude() * 100.0f; // Convert to cm? Check library units.
  
  // NOTE: You need to remove gravity from 'faz' and rotate it to world frame 
  // for the Kalman filter 'a' input. Passing raw 'faz' is usually wrong 
  // unless the device is perfectly flat.
  // kalman.Update(baroAlt, faz, 0.01f, &estimatedZ, &estimatedV); 

  Serial.print(angleX);
  Serial.print("\t");
  Serial.print(angleY);
  Serial.print("\t");
  Serial.print(angleZ);
  Serial.print("\n");
}