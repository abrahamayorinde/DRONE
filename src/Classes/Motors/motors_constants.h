#ifndef MOTORS_CONSTANTS_H
#define MOTORS_CONSTANTS_H
/********************MOTOR CONST*****************/
//const int MAX_DUTY_CYCLE_COUNT = 2000;

const float ThrottleMax = 1999;
const float ThrottleCutOff = 1000;
const float ThrottleIdle = 1050;

const float in_Min = 1000;
const float in_Max = 2000;
const float out_Min = 125;
const float out_Max = 250;

/*
    ONESHOT125 protocol uses a pwm signal with a minimum on time of
    125 milliseconds and a max on time of 250 milliseconds.
    The frequency of the pwm signal can vary between 500Hz to 4000Hz.
 */
const float MAX_ON_TIME_MSEC_PWM = 250;
const float MIN_ON_TIME_MSEC_PWM = 125;
const float ON_TIME_RANGE_MSEC_PWM = 125;

/*
    Teensy IO pins for each motor of quadcopter
 */
const int8_t MOTOR_FRONT_RIGHT  = 2;//input 4 on ESC
const int8_t MOTOR_FRONT_LEFT   = 4;//Input 2 on ESC
const int8_t MOTOR_REAR_RIGHT   = 6;//Input 3 on ESC
const int8_t MOTOR_REAR_LEFT    = 8;//Input 1 on ESC
#endif //MOTORS_CONSTANTS_H
