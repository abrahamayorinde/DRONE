#ifndef MPU92_65_CONSTANTS_H
#define MPU92_65_CONSTANTS_H

#include <cstdint>
const float ax_min = -2023;// -7980;//-8192;
const float ax_max =  2048;//  8390;//8192;

const float ay_min = -2043;// -8360;//-8192;
const float ay_max =  2055;//  8000;//8192;

const float az_min = -2048;// -8900;//16384;
const float az_max =  2092;// 7650;//24576;

const float ax_offset = (ax_min + ax_max)/2;
const float ay_offset = (ay_min + ay_max)/2;
const float az_offset = (az_min + az_max)/2;
const float x_scale = 2/(ax_max - ax_min);
const float y_scale = 2/(ay_max - ay_min);
const float z_scale = 2/(az_max - az_min);

const float GYRO_RESOLUTION = 16.375;//65.5; //degrees per second per count
const uint8_t interruptPin = 9;//11;


//const int MAX_DUTY_CYCLE_COUNT = 2000;

const float ThrottleMax = 1999;
const float ThrottleCutOff = 1000;
const float ThrottleIdle = 1050;//1180

const int in_Min = 1000;
const int in_Max = 2000;
const int out_Min = 125;//31.875;//31.875 MIN
const int out_Max = 250;//50.000;//63.750 MAX

const int MOTOR_FRONT_RIGHT  = 2;//8; //on teensy 4
const int MOTOR_FRONT_LEFT   = 4;//14;//on teensy 2
const int MOTOR_REAR_RIGHT   = 6;//17;//on teensy 3
const int MOTOR_REAR_LEFT    = 8;//20;//on teensy

constexpr float IMU_FILTER_ACCEL = 0.14;//0.003045902795; /* Equation for filter at loop rate of .001 or 1/1000 =>(1/1000)/( ( 1/(2*PI*10) ) + (1/1000) )*/
constexpr float IMU_FILTER_GYRO = 0.1;

const float maxRoll = 30.0;
const float maxPitch = 30.0;
const float maxYaw = 160.0;

const double IMU_CALIBRATION_COUNT = 1000;
#endif // MPU92_65_CONSTANTS_H

