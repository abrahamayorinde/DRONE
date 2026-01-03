#ifndef MPU92_65_CONSTANTS_H
#define MPU92_65_CONSTANTS_H

#include <cstdint>
#include <stdint.h>

const int32_t ax_min = -2023;// -7980;//-8192;
const int32_t ax_max =  2048;//  8390;//8192;

const int32_t ay_min = -2043;// -8360;//-8192;
const int32_t ay_max =  2055;//  8000;//8192;

const int32_t az_min = -2048;// -8900;//16384;
const int32_t az_max =  2092;// 7650;//24576;

const float_t ax_offset = (ax_min + ax_max)/2;
const float_t ay_offset = (ay_min + ay_max)/2;
const float_t az_offset = (az_min + az_max)/2;
const float_t x_scale = 2/(ax_max - ax_min);
const float_t y_scale = 2/(ay_max - ay_min);
const float_t z_scale = 2/(az_max - az_min);

const float GYRO_RESOLUTION = 16.375;//65.5; //degrees per second per count
const uint8_t interruptPin = 9;//11;




constexpr float IMU_FILTER_ACCEL = 0.14;//0.003045902795; /* Equation for filter at loop rate of .001 or 1/1000 =>(1/1000)/( ( 1/(2*PI*10) ) + (1/1000) )*/
constexpr float IMU_FILTER_GYRO = 0.1;

const float maxRoll = 30.0;
const float maxPitch = 30.0;
const float maxYaw = 160.0;

const double IMU_CALIBRATION_COUNT = 1000;

const int16_t PITCH_RATE_MAX = 240;
const int16_t PITCH_RATE_MIN = -240;

const int16_t ROLL_RATE_MAX = 240;
const int16_t ROLL_RATE_MIN = -240;


/********************MOTOR CONST*****************/
//const int MAX_DUTY_CYCLE_COUNT = 2000;

const int16_t ThrottleMax = 1999;
const int16_t ThrottleCutOff = 1000;
const int16_t ThrottleIdle = 1050;//1180

const int16_t in_Min = 1000;
const int16_t in_Max = 2000;
const int16_t out_Min = 125;
const int16_t out_Max = 250;

const int8_t MOTOR_FRONT_RIGHT  = 2;//input 4 on ESC
const int8_t MOTOR_FRONT_LEFT   = 4;//Input 2 on ESC
const int8_t MOTOR_REAR_RIGHT   = 6;//Input 3 on ESC
const int8_t MOTOR_REAR_LEFT    = 8;//Input 1 on ESC

/*
    ONESHOT125 protocol uses a pwm signal with a minimum on time of
    125 milliseconds and a max on time of 250 milliseconds.
    The frequency of the pwm signal can vary between 500Hz to 4000Hz.
 */
const float MAX_ON_TIME_MSEC_PWM = 250;
const float MIN_ON_TIME_MSEC_PWM = 125;
const float ON_TIME_RANGE_MSEC_PWM = 125; // OK
#endif // MPU92_65_CONSTANTS_H

