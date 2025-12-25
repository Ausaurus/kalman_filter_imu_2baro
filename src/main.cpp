#include "Arduino.h"
#include "ICM42688.h"
#include "MS5611.h"
#include "KalmanFilter.h"
#include "madgwickFilter.h"

// --- Helper Functions ---
float imu_GravityCompensatedAccel(float ax, float ay, float az, float *q);
float getMedian(float *array, int size);


ICM42688 IMU(Wire, 0x68);
MS5611 baro1(0x77);
MS5611 baro2(0x76);
KalmanFilter kalman;

// TUNING: High Z_VARIANCE = Smoother Velocity (but slower altitude response)
#define Z_VARIANCE 1000.0f      
#define ZACCEL_VARIANCE 100.0f
#define ZACCELBIAS_VARIANCE 1.0f
#define MEDIAN_SIZE 5

// --- Globals ---
float quat[4] = {1.0f,0.0f,0.0f,0.0f};
float beta = 0.6f;
float accelBias = 0.0f;
float smoothedAlt = 0.0f; // NEW: For pre-filtering barometer
float baroBuffer[MEDIAN_SIZE];
int bufferIndex = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000);

  if (IMU.begin() < 0) { Serial.println("IMU Fail"); while(1); }
  
  Wire.begin();
  Wire.setClock(100000); // Good idea for stability
  baro1.begin();
  baro2.begin();
  pinMode(LED_BUILTIN, OUTPUT);

  // --- Calibration ---
  Serial.println("Calibrating... DO NOT MOVE");
  digitalWrite(LED_BUILTIN, HIGH);
  
  float accelSum = 0;
  float baroSum = 0;
  int samples = 200;
  
  // FIX 2: Prime the buffer so it doesn't start at 0
  baro1.read(); baro2.read();
  float startAlt = (baro1.getAltitude() + baro2.getAltitude()) / 2.0f;
  for(int i=0; i<MEDIAN_SIZE; i++) {
    baroBuffer[i] = startAlt;
  }

  for (int i = 0; i < samples; i++) {
    // ... (Your existing calibration loop is fine) ...
    IMU.getAGT();
    baro1.read(); baro2.read();
    
    // Madgwick Logic ...
    float fax = IMU.accX();
    float fay = IMU.accY();
    float faz = IMU.accZ();
    float fgx = IMU.gyrX() * (PI / 180.0f);
    float fgy = IMU.gyrY() * (PI / 180.0f);
    float fgz = IMU.gyrZ() * (PI / 180.0f);
    if (abs(fgz) < 0.02f) fgz = 0;
    imu_filter(fax, fay, faz, fgx, fgy, fgz);

    float q[4] = {q_est.q1, q_est.q2, q_est.q3, q_est.q4};
    accelSum += imu_GravityCompensatedAccel(fax, fay, faz, q);
    
    // Keep priming buffer during calibration to be safe
    float avgRaw = (baro1.getAltitude() + baro2.getAltitude()) / 2.0f;
    baroBuffer[bufferIndex] = avgRaw;
    bufferIndex = (bufferIndex + 1) % MEDIAN_SIZE;
    
    baroSum += avgRaw; // Accumulate for initial Kalman setting
    delay(10);
  }
  
  accelBias = accelSum / samples;
  float initialAltCm = (baroSum / samples) * 100.0f; // Ensure this is CM
  
  kalman.Configure(Z_VARIANCE, ZACCEL_VARIANCE, ZACCELBIAS_VARIANCE, initialAltCm, 0.0f, 0.0f);
  
  digitalWrite(LED_BUILTIN, LOW);
  Serial.print("Bias: "); Serial.println(accelBias, 4);
}

void loop()
{
  static float zTrack = 0, vTrack = 0;
  static unsigned long lastMicros = 0;
  
  if (micros() - lastMicros < 10000) return;
  float dt = (micros() - lastMicros) / 1000000.0f;
  lastMicros = micros();

  // 1. Read Sensors
  baro1.read(); baro2.read();
  IMU.getAGT();
  
  // ... (Your existing IMU read / Madgwick logic is correct) ...
  float fax = IMU.accX();
  float fay = IMU.accY();
  float faz = IMU.accZ();
  float fgx = IMU.gyrX() * (PI / 180.0f);
  float fgy = IMU.gyrY() * (PI / 180.0f);
  float fgz = IMU.gyrZ() * (PI / 180.0f);
  if (abs(fgz) < 0.02f) fgz = 0.0f; 
  imu_filter(fax, fay, faz, fgx, fgy, fgz);
  
  // Accel Processing
  float quaternion[4] = {q_est.q1, q_est.q2, q_est.q3, q_est.q4};
  float raw_vertical_g = imu_GravityCompensatedAccel(fax, fay, faz, quaternion);
  float corrected_vertical_g = raw_vertical_g - accelBias;
  if (abs(corrected_vertical_g) < 0.05f) corrected_vertical_g = 0.0f;
  float linear_accel_cm_s2 = corrected_vertical_g * 980.665f;

  // 2. Baro Median Filter
  float rawAlt = (baro1.getAltitude() + baro2.getAltitude()) / 2.0f; // Meters
  
  baroBuffer[bufferIndex] = rawAlt;
  bufferIndex = (bufferIndex + 1) % MEDIAN_SIZE;
  
  float cleanAltMeters = getMedian(baroBuffer, MEDIAN_SIZE); // Still Meters!

  // 3. Update Kalman (FIX 1: CONVERT TO CM)
  kalman.Update(cleanAltMeters * 100.0f, linear_accel_cm_s2, dt, &zTrack, &vTrack);

  // 4. Print for Debug
  Serial.print(zTrack);
  Serial.print("\t");
  Serial.println(vTrack);
}

float imu_GravityCompensatedAccel(float ax, float ay, float az, float *q)
{
  return 2.0f * (q[1] * q[3] - q[0] * q[2]) * ax + 
         2.0f * (q[0] * q[1] + q[2] * q[3]) * ay +
         (q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]) * az - 1.0f;
}

float getMedian(float *array, int size) {
  float sorted[size];
  for (int i = 0; i < size; i++) sorted[i] = array[i];
  
  // Simple Bubble Sort
  for (int i = 0; i < size - 1; i++) {
    for (int j = 0; j < size - i - 1; j++) {
      if (sorted[j] > sorted[j + 1]) {
        float temp = sorted[j];
        sorted[j] = sorted[j + 1];
        sorted[j + 1] = temp;
      }
    }
  }
  return sorted[size / 2];
}