#ifndef PID_CONSTANTS_H
#define PID_CONSTANTS_H

    //const float PVelocityVertical = 1;
    //const float IVelocityVertical = 0;
    //const float DVelocityVertical = 0;
    //const float FeedForwardGainVertical = 1.0;
    ///****************************************/
    //const float PAngleRoll =  0.10;//0.2;
    //const float IAngleRoll =  0.15;//0.3;
    //const float DAngleRoll =  .025;//.05;
    //const float FeedForwardGainRoll = 1.0;
    //
    //const float Roll_Rate_Damping = 0.9;
    //const float PRateRoll =  .0750;//.15;
    //const float IRateRoll =  .2000;//.2;
    //const float DRateRoll =  .0001;//0.0002;
    //const float FeedForwardGainRollRate = 1.0;
    ///****************************************/
    //const float PAnglePitch = 0.10;//0.2;
    //const float IAnglePitch = 0.15;//0.3;
    //const float DAnglePitch = .025;//.05;
    //const float FeedForwardGainPitch = 1.0;
    //
    //const float Pitch_Rate_Damping = 0.9;
    //const float PRatePitch =  .0750;//.15;
    //const float IRatePitch =  .2000;//.2;
    //const float DRatePitch =  .0001;//0.0002;
    //const float FeedForwardGainPitchRate = 1.0;
    ///****************************************/
    //const float PRateYaw =    0.15000;//0.3;
    //const float IRateYaw =    0.05000;//0.05;
    //const float DRateYaw =    0.00015;//.000015;
    //const float FeedForwardGainYawRate = 1.0;
    //
    //
    //const float integral_Windup = 30;
    //const float loop_Saturation = 1000;
    //const float pitch_integral_Windup = 25;
    //const float roll_integral_Windup = 25;


const float PVelocityVertical = 1;
const float IVelocityVertical = 0;
const float DVelocityVertical = 0;
const float FeedForwardGainVertical = 1.0;
/****************************************/
const float PAngleRoll =  0.08; //0.200;
const float IAngleRoll =  0.200;//0.300;
const float DAngleRoll =  0;    //.0500;
const float FeedForwardGainRoll = 1.0;

const float Roll_Rate_Damping = 0.9;
const float PRateRoll =  .06;     //.1500;
const float IRateRoll =  .1500;   //.2000;
const float DRateRoll =  0.000010;//.0002;
const float FeedForwardGainRollRate = 1.0;
/****************************************/
const float PAnglePitch = 0.08; //0.200;
const float IAnglePitch = 0.200;//0.300;
const float DAnglePitch = 0;    //.0500;
const float FeedForwardGainPitch = 1.0;

const float Pitch_Rate_Damping = 0.9;
const float PRatePitch =  .06;     //.1500;
const float IRatePitch =  .1500;   //.2000;
const float DRatePitch =  0.000010;//.0002;
const float FeedForwardGainPitchRate = 1.0;
/****************************************/
const float PRateYaw =    0.0750000;//0.15000
const float IRateYaw =    0.0125000;//.05
const float DRateYaw =    0.0000375;//.00015
const float FeedForwardGainYawRate = 1.0;


const float integral_Windup = 30;
const float loop_Saturation = 1000;
const float pitch_integral_Windup = 25;
const float roll_integral_Windup = 25;
#endif // PID_CONSTANTS_H

