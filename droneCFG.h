
#ifndef DRONE_CFG_H
#define DRONE_CFG_H

#include "src/Classes/Filter/filters.h"
#include "src/Classes/PID/pid.h"
#include "src/Classes/Remote/fsi6.h"
#include "src/Classes/IMU/mpu92_65.h"

MPU_92_65 imu_6500;

fsi6 RemoteControl;

SecondOrderFilter Pitch_2nd;
SecondOrderFilter Roll_2nd;

PID elevation_control(RemoteControl.InputValue[Velocity], PVelocityVertical, IVelocityVertical, DVelocityVertical, FeedForwardGainVertical, integral_Windup, loop_Saturation, false);
PID pitch_control(RemoteControl.InputValue[Velocity], PAnglePitch, IAnglePitch, DAnglePitch,FeedForwardGainPitch, pitch_integral_Windup, loop_Saturation, false);
PID pitch_rate_control(RemoteControl.InputValue[Velocity], PRatePitch, IRatePitch, DRatePitch, FeedForwardGainPitchRate, pitch_integral_Windup, loop_Saturation, false);
PID roll_control(RemoteControl.InputValue[Velocity], PAngleRoll, IAngleRoll, DAngleRoll,FeedForwardGainRoll, roll_integral_Windup, loop_Saturation, false);
PID roll_rate_control(RemoteControl.InputValue[Velocity], PRateRoll, IRateRoll, DRateRoll, FeedForwardGainRollRate, roll_integral_Windup, loop_Saturation, false);
PID yaw_rate_control(RemoteControl.InputValue[Velocity], PRateYaw, IRateYaw, DRateYaw, FeedForwardGainYawRate, integral_Windup, loop_Saturation, false);

MadgwickFilter Madgwick;



#endif // DRONE_CFG_H


