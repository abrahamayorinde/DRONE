
#ifndef DRONE_CFG_H
#define DRONE_CFG_H

#include "src/Classes/Filter/filters.h"
#include "src/Classes/PID/pid.h"
#include "src/Classes/Remote/fsi6.h"
#include "src/Classes/IMU/mpu92_65.h"
#include "src/Classes/Motors/motors.h"

//declare sensor object from mpu92_65 header file
MPU_92_65 imu_6500;

//declare fsi6 object from respective header file
fsi6 RemoteControl;

//declare filter objects from filter.h file
SecondOrderFilter Pitch_2nd;
SecondOrderFilter Roll_2nd;

//declare 
PID elevation_control(RemoteControl.InputValue[Velocity], PVelocityVertical, IVelocityVertical, DVelocityVertical, FeedForwardGainVertical, integral_Windup, loop_Saturation, false);
PID pitch_control(RemoteControl.InputValue[Velocity], PAnglePitch, IAnglePitch, DAnglePitch,FeedForwardGainPitch, pitch_integral_Windup, loop_Saturation, false);
PID pitch_rate_control(RemoteControl.InputValue[Velocity], PRatePitch, IRatePitch, DRatePitch, FeedForwardGainPitchRate, pitch_integral_Windup, loop_Saturation, false);
PID roll_control(RemoteControl.InputValue[Velocity], PAngleRoll, IAngleRoll, DAngleRoll,FeedForwardGainRoll, roll_integral_Windup, loop_Saturation, false);
PID roll_rate_control(RemoteControl.InputValue[Velocity], PRateRoll, IRateRoll, DRateRoll, FeedForwardGainRollRate, roll_integral_Windup, loop_Saturation, false);
PID yaw_rate_control(RemoteControl.InputValue[Velocity], PRateYaw, IRateYaw, DRateYaw, FeedForwardGainYawRate, integral_Windup, loop_Saturation, false);

MadgwickFilter Madgwick;


motor_obj front_right(MOTOR_FRONT_RIGHT);

motor_obj front_left(MOTOR_FRONT_LEFT);

motor_obj rear_right(MOTOR_REAR_RIGHT);

motor_obj rear_left(MOTOR_REAR_LEFT);

struct motors
{
  motor_obj motor;
  uint32_t duration;
};

motors quadcopter[4] = {{front_right, 0}, {front_left, 0}, {rear_right,0}, {rear_left,0}};
#endif // DRONE_CFG_H


