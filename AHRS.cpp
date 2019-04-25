#include "AHRS.h"
AHRS::AHRS(Adafruit_LSM9DS1 * the_MARG, float sampleFreq):
 _MARG(the_MARG), 
 deltat_mu(0), 
 dt(10000), 
 aJerk{}, 
 gJerk{},
 accIn{},
 accOut{},
 orientation{},
 q{1,0,0,0}{
}
void AHRS::setup()
{
   Serial.println("LSM9DS1 basic functioning");
  
  // Try to initialise and warn if we couldn't detect the chip
  if (!_MARG->begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS1 9DOF");

// helper to just set the default scaling we want, see above!
// 1.) Set the accelerometer range
//_MARG->setupAccel(_MARG->LSM9DS1_ACCELRANGE_2G);
//  _MARG->setupAccel(_MARG->LSM9DS1_ACCELRANGE_4G);
  _MARG->setupAccel(_MARG->LSM9DS1_ACCELRANGE_8G);
//_MARG->setupAccel(_MARG->LSM9DS1_ACCELRANGE_16G);

// 2.) Set the magnetometer sensitivity
//  _MARG->setupMag(_MARG->LSM9DS1_MAGGAIN_4GAUSS);
  _MARG->setupMag(_MARG->LSM9DS1_MAGGAIN_8GAUSS);
//  _MARG->setupMag(_MARG->LSM9DS1_MAGGAIN_12GAUSS);
//_MARG->setupMag(_MARG->LSM9DS1_MAGGAIN_16GAUSS);

// 3.) Setup the gyroscope
//  _MARG->setupGyro(_MARG->LSM9DS1_GYROSCALE_245DPS);
  _MARG->setupGyro(_MARG->LSM9DS1_GYROSCALE_500DPS);
//_MARG->setupGyro(_MARG->LSM9DS1_GYROSCALE_2000DPS);
}

void AHRS::update(boolean calibrate){
  _MARG->read();
  _MARG->getEvent(&a, &m, &g, &temp); 
  updateFilter();
  if(micros() - lastJerk > dt) {
    lastJerk = micros();
    updateAccJerk();
    updateGyroJerk();
  }
  if(calibrate) print2MotionCal();
  deltat_mu = a.timestamp - lastRead;
  lastRead = a.timestamp;

  MadgwickQuaternionUpdate(a.acceleration.x, a.acceleration.y, a.acceleration.z, g.gyro.x * PI / 180.0f, g.gyro.y * PI / 180.0f, g.gyro.z * PI / 180.0f, m.magnetic.x, m.magnetic.y, m.magnetic.z,deltat_mu/1000000.0f);
  computeAngles();
}

void AHRS::MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat){
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
//    float _2q1q2 = 2.0f * q1 * q2; //an
    float _2q1q3 = 2.0f * q1 * q3;
//    float _2q1q4 = 2.0f * q1 * q4; //an
//    float _2q2q3 = 2.0f * q2 * q3; //an
//    float _2q2q4 = 2.0f * q2 * q4; //an
    float _2q3q4 = 2.0f * q3 * q4;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    //normAccel = norm; //an
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * mx;
    _2q1my = 2.0f * q1 * my;
    _2q1mz = 2.0f * q1 * mz;
    _2q2mx = 2.0f * q2 * mx;
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
    norm = 1.0f/norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

    // Integrate to yield quaternion

    q1 += qDot1 * deltat;
    q2 += qDot2 * deltat;
    q3 += qDot3 * deltat;
    q4 += qDot4 * deltat;
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;


}

void AHRS::computeAngles()
{
  float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];   // short name local variable for readability
  orientation[0]  = asinf(-2.0f * (q1*q3 - q0*q2)); // Pitch
  orientation[0] *= 180.0f / PI;

  orientation[1]  = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2); // Roll
  orientation[1] *= 180.0f / PI; 
  orientation[1] -= DECLINATION; // Declination at Montreal, QC.-14.49

  orientation[2]  = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3); // Yaw
  orientation[2]  *= 180.0f / PI;
}

void AHRS::print2MotionCal(){
  // Print the sensor data
  
  Serial.print("Raw:");
  Serial.print((int)_MARG->accelData.x);
  Serial.print(',');
  Serial.print((int)_MARG->accelData.y);
  Serial.print(',');
  Serial.print((int)_MARG->accelData.z);
  Serial.print(',');
  Serial.print((int)_MARG->gyroData.x);
  Serial.print(',');
  Serial.print((int)_MARG->gyroData.y);
  Serial.print(',');
  Serial.print((int)_MARG->gyroData.z);
  Serial.print(',');
  Serial.print((int)_MARG->magData.x);
  Serial.print(',');
  Serial.print((int)_MARG->magData.y);
  Serial.print(',');
  Serial.print((int)_MARG->magData.z);
  Serial.println();  
}

float * AHRS::getOrientation(){
  return orientation;
}

float AHRS::getAccMag(){
  float x = a.acceleration.x; float y = a.acceleration.y; float z= a.acceleration.z;
  return  sqrt(x*x + y*y + z*z)/63.0f;
}

void AHRS::updateAccJerk(){
  static float xm1, ym1, zm1;   
  // float x = a.acceleration.x; float y = a.acceleration.y; float z= a.acceleration.z;
  float x = accOut[0][0]; float y = accOut[1][0]; float z= accOut[2][0];
  float xdiff = x - xm1; float ydiff = y - ym1; float zdiff = z-zm1;
  xm1 = x; ym1 =y; zm1 = z;
  aJerk[0] = xdiff; aJerk[1] = ydiff; aJerk[2] = zdiff;
}

void AHRS::updateGyroJerk(){
  static float xm1, ym1, zm1;
  float x = g.gyro.x; float y = g.gyro.y; float z= g.gyro.z;
  float xdiff = x - xm1; float ydiff = y - ym1; float zdiff = z-zm1;
  xm1 = x; ym1 =y; zm1 = z;
  gJerk[0] = xdiff; gJerk[1] = ydiff; gJerk[2] = zdiff;
}

float AHRS::getAccJerkMag(){
  return sqrt(pow(aJerk[0],2) + pow(aJerk[1],2) + pow(aJerk[2],2))/30.0f;  
}

float AHRS::getGyroMag(){
  float x = g.gyro.x; float y = g.gyro.y; float z= g.gyro.z;
  return  (float)sqrt(x*x + y*y + z*z)/500.0f;
}

float AHRS::getGyroJerkMag(){
  return sqrt(pow(gJerk[0],2) + pow(gJerk[1],2) + pow(gJerk[2],2))/250.0f;
}

float * AHRS::getAcc(){
  acc[0] = a.acceleration.x; acc[1] = a.acceleration.y; acc[2] = a.acceleration.z; 
  return  acc;
}

float * AHRS::getGyro(){
  gyro[0] = g.gyro.x; gyro[1] = g.gyro.y; gyro[2] = g.gyro.z; 
  return  gyro;
}

void AHRS::updateFilter(){
  static float tmp;
  accIn[0][0] = a.acceleration.x; accIn[1][0] = a.acceleration.y; accIn[2][0] = a.acceleration.z;
  for(int i = 0; i < 3; i++){
      tmp = accIn[i][0] * ff[0] + accIn[i][1] *ff[1] + accIn[i][2] * ff[2] - accOut[i][1] * fb[1] - accOut[i][2] * fb[2];
      // Update state variables
        accIn[i][2] = accIn[i][1]; accIn[i][1] = accIn[i][0];
        accOut[i][2] = accOut[i][1]; accOut[i][1] = accOut[i][0]; accOut[i][0] = tmp;
  }
}






