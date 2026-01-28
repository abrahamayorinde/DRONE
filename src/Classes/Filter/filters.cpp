
// filter.cpp/Users/abrahamayorinde/Documents/Arduino/libraries/FILTERS/filters.cpp
#include "filters.h"
#include <math.h>
#include <Wire.h>
//using namespace std;

FirstOrderFilter::FirstOrderFilter()
{
    this->a = 0;
    this->tau = 0.015;//0.075;
    this->compfilter_xangleC = 0;
    this->comp_start = false;
}


FirstOrderFilter::FirstOrderFilter(float Tau)
{
    this->a = 0;
    this->tau = Tau;//0.015;//0.075;
    this->compfilter_xangleC = 0;
    this->comp_start = false;
}


float FirstOrderFilter::Complementary(float newAngle, float newRate, float looptime)
{
    float dtC = float(looptime);//1000.0;

    if (comp_start == true)
    {
        comp_start = false;
        compfilter_xangleC = newAngle;
        return compfilter_xangleC;
    }

    a=tau/(tau+dtC);
    compfilter_xangleC = a * (compfilter_xangleC + newRate * dtC) + (1-a) * (newAngle);
    
    return compfilter_xangleC;
}


SecondOrderFilter::SecondOrderFilter()
{
    this->out_angle = 0;
    this->gyro_filter = 0;
    this->filter_x1 = 0;
    this->filter_x2 = 0;
}


SecondOrderFilter::SecondOrderFilter(float Bandwidth)
{
    this->k = Bandwidth;
    this->out_angle = 0;
    this->gyro_filter = 0;
    this->filter_x1 = 0;
    this->filter_x2 = 0;
}


float SecondOrderFilter::Complementary(float accel_angle, float newRate, float timeDelta)
{
    //timeDelta //1000.0;
    int k=10;//10(bandiwdth in Hz) //tried 500
    filter_x1 = (accel_angle - out_angle)*k*k;
    gyro_filter = timeDelta*filter_x1 + gyro_filter;
    filter_x2 = gyro_filter + (accel_angle - out_angle)*2*k + newRate;
    out_angle = timeDelta*filter_x2 + out_angle;

    return out_angle;
}


double KalmanFilter::Kalman(double angle, double gyroRate, double accelAngle, float deltaT)
{
  // Predict
  rate = gyroRate - bias;
  //deltaT = deltaT / 1000000;
  angle += deltaT * rate;
 
  P[0][0] += deltaT * (deltaT * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= deltaT * P[1][1];
  P[1][0] -= deltaT * P[1][1];
  P[1][1] += Q_bias * deltaT;
 
  // Update
  double S = P[0][0] + R_measure; // Estimate error
  double K[2];                    // Kalman gain
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;
 
  double y = accelAngle - angle; // Angle difference
  angle += K[0] * y;
  bias += K[1] * y;
 
  double P00_temp = P[0][0];
  double P01_temp = P[0][1];
 
  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;
  
  return angle;
}


MahonyFilter::MahonyFilter(float Kp_Mahony, float Ki_Mahony)
{
    this->Kp = Kp_Mahony;
    this->Ki = Ki_Mahony;
    this->q[0] = 1;
    this->q[1] = 0;
    this->q[2] = 0;
    this->q[3] = 0;
}


void MahonyFilter::Mahony_update(float ax, float ay, float az, float gx, float gy, float gz, float deltat)
{
  float recipNorm;
  float vx, vy, vz;
  float ex, ey, ez;  //error terms
  float qa, qb, qc;
  static float ix = 0.0, iy = 0.0, iz = 0.0;  //integral feedback terms
  float tmp;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  tmp = ax * ax + ay * ay + az * az;
  if (tmp > 0.0)
  {
    // Normalise accelerometer (assumed to measure the direction of gravity in body frame)
    recipNorm = 1.0 / sqrt(tmp);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity in the body frame (factor of two divided out)
    vx = q[1] * q[3] - q[0] * q[2];  //to normalize these terms, multiply each by 2.0
    vy = q[0] * q[1] + q[2] * q[3];
    vz = q[0] * q[0] - 0.5f + q[3] * q[3];

    // Error is cross product between estimated and measured direction of gravity in body frame
    // (half the actual magnitude)
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    // Compute and apply to gyro term the integral feedback, if enabled
    if (Ki > 0.0f)
    {
      ix += Ki * ex * deltat;  // integral error scaled by Ki
      iy += Ki * ey * deltat;
      iz += Ki * ez * deltat;
      gx += ix;  // apply integral feedback
      gy += iy;
      gz += iz;
    }

    // Apply proportional feedback to gyro term
    gx += Kp * ex;
    gy += Kp * ey;
    gz += Kp * ez;
  }

  // Integrate rate of change of quaternion, q cross gyro term
  deltat = 0.5 * deltat;
  gx *= deltat;   // pre-multiply common factors
  gy *= deltat;
  gz *= deltat;
  qa = q[0];
  qb = q[1];
  qc = q[2];
  q[0] += (-qb * gx - qc * gy - q[3] * gz);
  q[1] += (qa * gx + qc * gz - q[3] * gy);
  q[2] += (qa * gy - qb * gz + q[3] * gx);
  q[3] += (qa * gz + qb * gy - qc * gx);

  // renormalise quaternion
  recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  q[0] = q[0] * recipNorm;
  q[1] = q[1] * recipNorm;
  q[2] = q[2] * recipNorm;
  q[3] = q[3] * recipNorm;

  roll  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
  pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
  //conventional yaw increases clockwise from North. Not that the MPU-6050 knows where North is.
  yaw   = -atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - (q[2] * q[2] + q[3] * q[3]));
  // to degrees
    yaw   *= 180.0 / M_PI;//PI;
  if (yaw < 0)
  {
    yaw += 360.0;
  } //compass circle
  pitch *= 180.0 / M_PI;
  roll *= 180.0 / M_PI;
}

void MahonyFilter::getEuler(float &Pitch, float& Yaw, float &Roll)
{
    Pitch = pitch;
    Yaw = yaw;
    Roll = roll;
}


void MahonyFilter::getQuaternion(float& z, float& i, float& j, float& k)
{
    z = q[0];
    i = q[1];
    j = q[2];
    k = q[3];
}


MadgwickFilter::MadgwickFilter(float filter_gain)
{
    this->gain_Madgwick = filter_gain;
    this->q[0] = 1;
    this->q[1] = 0;
    this->q[2] = 0;
    this->q[3] = 0;
}


MadgwickFilter::MadgwickFilter()
{
    this->gain_Madgwick = 0.04;
    this->q[0] = 1;
    this->q[1] = 0;
    this->q[2] = 0;
    this->q[3] = 0;
}


void MadgwickFilter::Madgwick9DOF(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float invSampleFreq)
{
  //DESCRIPTION: Attitude estimation through sensor fusion - 9DOF
  /*
   * This function fuses the accelerometer gyro, and magnetometer readings AccX, AccY, AccZ, GyroX, GyroY, GyroZ, MagX, MagY, and MagZ for attitude estimation.
   * Don't worry about the math. There is a tunable parameter gain_Madgwick in the user specified variable section which basically
   * adjusts the weight of gyro data in the state estimate. Higher beta leads to noisier estimate, lower
   * beta leads to slower to respond estimate. It is currently tuned for 2kHz loop rate. This function updates the roll_IMU,
   * pitch_IMU, and yaw_IMU variables which are in degrees. If magnetometer data is not available, this function calls Madgwick6DOF() instead.
   */
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

  //use 6DOF algorithm if MPU6050 is being used
  #if defined USE_MPU6050_I2C
    Madgwick::Madgwick6DOF(gx, gy, gz, ax, ay, az, invSampleFreq);
    return;
  #endif
  
  //Use 6DOF algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
  
  if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
  {
    Madgwick6DOF(gx, gy, gz, ax, ay, az, invSampleFreq);
    return;
  }
  
  //Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q[1] * gx - q[2] * gy - q[3] * gz);
  qDot2 = 0.5f * (q[0] * gx + q[2] * gz - q[3] * gy);
  qDot3 = 0.5f * (q[0] * gy - q[1] * gz + q[3] * gx);
  qDot4 = 0.5f * (q[0] * gz + q[1] * gy - q[2] * gx);

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
  {

    //Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    //Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q[0] * mx;
    _2q0my = 2.0f * q[0] * my;
    _2q0mz = 2.0f * q[0] * mz;
    _2q1mx = 2.0f * q[1] * mx;
    _2q0 = 2.0f * q[0];
    _2q1 = 2.0f * q[1];
    _2q2 = 2.0f * q[2];
    _2q3 = 2.0f * q[3];
    _2q0q2 = 2.0f * q[0] * q[2];
    _2q2q3 = 2.0f * q[2] * q[3];
    q0q0 = q[0] * q[0];
    q0q1 = q[0] * q[1];
    q0q2 = q[0] * q[2];
    q0q3 = q[0] * q[3];
    q1q1 = q[1] * q[1];
    q1q2 = q[1] * q[2];
    q1q3 = q[1] * q[3];
    q2q2 = q[2] * q[2];
    q2q3 = q[2] * q[3];
    q3q3 = q[3] * q[3];

    //Reference direction of Earth's magnetic field
    hx = mx * q0q0 - _2q0my * q[3] + _2q0mz * q[2] + mx * q1q1 + _2q1 * my * q[2] + _2q1 * mz * q[3] - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q[3] + my * q0q0 - _2q0mz * q[1] + _2q1mx * q[2] - my * q1q1 + my * q2q2 + _2q2 * mz * q[3] - my * q3q3;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q0mx * q[2] + _2q0my * q[1] + mz * q0q0 + _2q1mx * q[3] - mz * q1q1 + _2q2 * my * q[3] - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    //Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q[2] * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q[3] + _2bz * q[1]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q[2] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q[1] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q[3] * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q[2] + _2bz * q[0]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q[3] - _4bz * q[1]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q[2] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q[2] - _2bz * q[0]) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q[1] + _2bz * q[3]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q[0] - _4bz * q[2]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q[3] + _2bz * q[1]) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q[0] + _2bz * q[2]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q[1] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //Apply feedback step
    qDot1 -= gain_Madgwick * s0;
    qDot2 -= gain_Madgwick * s1;
    qDot3 -= gain_Madgwick * s2;
    qDot4 -= gain_Madgwick * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion
  q[0] += qDot1 * invSampleFreq;
  q[1] += qDot2 * invSampleFreq;
  q[2] += qDot3 * invSampleFreq;
  q[3] += qDot4 * invSampleFreq;

  //Normalise quaternion
  recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  q[0] *= recipNorm;
  q[1] *= recipNorm;
  q[2] *= recipNorm;
  q[3] *= recipNorm;
  
  //compute angles - NWU
  roll  = atan2(q[0]*q[1] + q[2]*q[3], 0.5f - q[1]*q[1] - q[2]*q[2])*57.29577951; //degrees
  pitch = -asin(-2.0f * (q[1]*q[3] - q[0]*q[2]))*57.29577951; //degrees
  yaw   = -atan2(q[1]*q[2] + q[0]*q[3], 0.5f - q[2]*q[2] - q[3]*q[3])*57.29577951; //degrees

}


void MadgwickFilter::Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq)
{
  //DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
  /*
   * See description of Madgwick() for more information. This is a 6DOF implimentation for when magnetometer data is not
   * available (for example when using the recommended MPU6050 IMU for the default setup).
   */
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  //Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q[1] * gx - q[2] * gy - q[3] * gz);
  qDot2 = 0.5f * (q[0] * gx + q[2] * gz - q[3] * gy);
  qDot3 = 0.5f * (q[0] * gy - q[1] * gz + q[3] * gx);
  qDot4 = 0.5f * (q[0] * gz + q[1] * gy - q[2] * gx);

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
  {
    //Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q[0];
    _2q1 = 2.0f * q[1];
    _2q2 = 2.0f * q[2];
    _2q3 = 2.0f * q[3];
    _4q0 = 4.0f * q[0];
    _4q1 = 4.0f * q[1];
    _4q2 = 4.0f * q[2];
    _8q1 = 8.0f * q[1];
    _8q2 = 8.0f * q[2];
    q0q0 = q[0] * q[0];
    q1q1 = q[1] * q[1];
    q2q2 = q[2] * q[2];
    q3q3 = q[3] * q[3];

    //Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q[1] - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q[2] + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q[3] - _2q1 * ax + 4.0f * q2q2 * q[3] - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); //normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //Apply feedback step
    qDot1 -= gain_Madgwick * s0;
    qDot2 -= gain_Madgwick * s1;
    qDot3 -= gain_Madgwick * s2;
    qDot4 -= gain_Madgwick * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion
  q[0] += qDot1 * invSampleFreq;
  q[1] += qDot2 * invSampleFreq;
  q[2] += qDot3 * invSampleFreq;
  q[3] += qDot4 * invSampleFreq;

  //Normalise quaternion
  recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  q[0] *= recipNorm;
  q[1] *= recipNorm;
  q[2] *= recipNorm;
  q[3] *= recipNorm;

  //Compute angles
  roll  = atan2(q[0]*q[1] + q[2]*q[3], 0.5f - q[1]*q[1] - q[2]*q[2])*57.29577951; //degrees
  pitch = -asin(constrain(-2.0f * (q[1]*q[3] - q[0]*q[2]),-0.999999,0.999999))*57.29577951; //degrees
  yaw   = -atan2(q[1]*q[2] + q[0]*q[3], 0.5f - q[2]*q[2] - q[3]*q[3])*57.29577951; //degrees
}


float MadgwickFilter::invSqrt(float x)
{
  //Fast inverse sqrt for madgwick filter
  
  unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
  float tmp = *(float*)&i;
  float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
  return y;
}


void MadgwickFilter::getEuler(float &Pitch, float& Yaw, float &Roll)
{
    Pitch = pitch;
    Yaw = yaw;
    Roll = roll;
}


void MadgwickFilter::getQuaternion(float& z, float& i, float& j, float& k)
{
    z = q[0];
    i = q[1];
    j = q[2];
    k = q[3];
}
