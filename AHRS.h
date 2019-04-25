#ifndef __AHRS_H__
#define __AHRS_H__
#pragma once

#include <Adafruit_LSM9DS1.h>
#include <math.h>

#define DECLINATION -14.34 // Declination (degrees) in Montreal, QC.
#define GyroMeasError PI * (120.0f / 180.0f)       // gyroscope measurement error in rads/s (shown as 3 deg/s)
#define GyroMeasDrift PI * (1.0f / 180.0f)      // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
#define beta sqrt(3.0f / 4.0f) * GyroMeasError   // compute beta
#define zeta sqrt(3.0f / 4.0f) * GyroMeasDrift   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value


class AHRS{
public:
	AHRS(Adafruit_LSM9DS1 * the_MARG, float sampleFreq);
  void update(boolean calibrate);
  void updateFilter();
  void updateAccJerk();
  void updateGyroJerk();
  void setup();
  void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);
  void computeAngles();
  void print2MotionCal();
  float * getAcc();
  float * getGyro();
  float * getOrientation();
  float getAccMag();
  float getAccJerkMag();
  float getGyroMag();
  float getGyroJerkMag();
  Adafruit_LSM9DS1* _MARG;
  sensors_event_t a, m, g, temp;
	long deltat_mu; // [mu s]
	long lastRead; // [mu s]
  long dt; // [mu s]
  long lastJerk;
  float q[4];
  float orientation[3]; //pitch, roll, yaw;
  float aJerk[3];
  float gJerk[3];
  float acc[3];
  float accIn[3][3];
  float accOut[3][3];
  float gyro[3];
  // Filter
  float ff[3] =  {0.5000,         0,   -0.5000};
  float fb[3] = {1.0000,    0.2212,    0.0000};

};

#endif
