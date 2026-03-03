#include <Wire.h>
#include <stdint.h>
#include "droneCFG.h"

#define MICROSEC_PER_SECOND 1000000

///////////////////////////////////   LOCAL   VARIABLES      ///////////////////////////////////
float deltaT_F = 0;
unsigned long previousTime = 0;
unsigned long currentStart = 0;

float DesiredVerticalVelocity, DesiredRollAngle, DesiredPitchAngle, DesiredRollRate, DesiredPitchRate, DesiredYawRate = {0};
float DesiredRollRateInput, DesiredPitchRateInput = {0};


float RollAngleError,  RollRateError  = {0};
float PitchAngleError, PitchRateError = {0};

float YawRateError = {0};

float InputPitch, InputRoll, InputYaw = {0};

float dutyCycleRearLeft, dutyCycleFrontLeft, dutyCycleRearRight, dutyCycleFrontRight = {0};
float rearLeftMotorCommand, frontLeftMotorCommand, rearRightMotorCommand, frontRightMotorCommand = {0};
///////////////////////////////////////////////////////////////////////////////////////////
IntervalTimer msec500Timer;
IntervalTimer msec1Timer;
IntervalTimer motorTimer;

uint32_t turn_on_time = 0;
uint32_t turn_off_time = 0;
void setup() 
{
  //Serial.begin(115200);

  imu_6500.checkIMUSPISensor();//verify that the sensor is working correctly.  If it is not, the software will not halt here.

  //Serial.println("Sensor check passed.");

  attachInterrupt(digitalPinToInterrupt(interruptPin), RemoteControl.isr, RISING);

  //Serial.println("Interrupt attach complete.");

  analogWriteResolution(8); //Resolution of data written to pinMode outputs is 8-bit

  //Serial.println("Write resolution (8) complete.");

  imu_6500.calibrateGyro();

  //Serial.println("Gyro offsets calculated.");

  RemoteControl.calibrateRemote();  //Calculate offsets with yaw, pitch and roll at center position and throttle at lowest position.

  //Serial.println("Remote offsets calculated.");

  MadgwickConverge(); //allow Madgwick filter to converge before utilizing the data in the control algorithm

  //Serial.println("Madgwick filter converged to stable value.");

  msec500Timer.begin(motor_isr_ON, 500);
}

void loop() 
{
  previousTime = currentStart;
  currentStart = micros();
  deltaT_F = ((float)(currentStart - previousTime)/MICROSEC_PER_SECOND); //convert from microseconds to seconds

  imu_6500.getSensorData();

  //Collect the desired roll angle from remote and while subtracting the offset
  DesiredRollAngle  =  (RemoteControl.InputValue[Roll] - RemoteControl.roll_offset)/ROLL_MARGIN;
  DesiredRollAngle  = maxRoll*constrain(DesiredRollAngle, -1, 1);

  //Collect the desired pitch angle from remote and while subtracting the offset
  DesiredPitchAngle = -(RemoteControl.InputValue[Pitch] - RemoteControl.pitch_offset)/PITCH_MARGIN;//-1 coeff pitch angle correponds to imu orientation
  DesiredPitchAngle = maxPitch*constrain(DesiredPitchAngle, -1, 1);

  //Collect the desired yaw rate  from remote and while subtracting the offset
  DesiredYawRate = -(RemoteControl.InputValue[Yaw] - RemoteControl.yaw_offset )/YAW_MARGIN;//-1 coeff yaw angle correponds to imu orientation
  DesiredYawRate = maxYaw*constrain(DesiredYawRate, -1, 1);

  //Collect the desired vertical velocity/thrust from remote and while subtracting the offset
  DesiredVerticalVelocity = (RemoteControl.InputValue[Velocity] - THROTTLE_RANGE)/THROTTLE_RANGE;      
  DesiredVerticalVelocity = constrain(DesiredVerticalVelocity, 0, 1);

  //Pass the raw imu data to the madgwick filter
  Madgwick.Madgwick6DOF(imu_6500.GyroX_Filt, -imu_6500.GyroY_Filt, -imu_6500.GyroZ_Filt, -imu_6500.AccX_Filt, imu_6500.AccY_Filt, imu_6500.AccZ_Filt,  deltaT_F);


  ////////////////////////////////////////////// ROLL  CONTROL /////////////////////////////////////////
  RollAngleError = DesiredRollAngle - ( Madgwick.roll);
  DesiredRollRate = roll_control.pid_equation(RollAngleError, deltaT_F);

  DesiredRollRate = constrain(DesiredRollRate, ROLL_RATE_MIN, ROLL_RATE_MAX);
  DesiredRollRateInput = (1 - Roll_Rate_Damping)*DesiredRollRateInput + Roll_Rate_Damping * DesiredRollRate;

  RollRateError = DesiredRollRateInput - imu_6500.GyroX;
  InputRoll = .01*roll_rate_control.pid_equation(RollRateError, deltaT_F);

  //////////////////////////////////////////////////////////////////////////////////////////////////////


  ///////////////////////////////////////////// PITCH  CONTROL /////////////////////////////////////////
  PitchAngleError = DesiredPitchAngle - ((-Madgwick.pitch) + 9.5);// ;
  DesiredPitchRate = pitch_control.pid_equation(PitchAngleError, deltaT_F);

  DesiredPitchRate = constrain(DesiredPitchRate, PITCH_RATE_MIN, PITCH_RATE_MAX);
  DesiredPitchRateInput = (1 - Pitch_Rate_Damping)*DesiredPitchRateInput + Pitch_Rate_Damping * DesiredPitchRate;

  PitchRateError = (DesiredPitchRateInput + imu_6500.GyroY);
  InputPitch = .01* pitch_rate_control.pid_equation(PitchRateError, deltaT_F);
  //////////////////////////////////////////////////////////////////////////////////////////////////////


  ///////////////////////////////////////////// YAW  CONTROL ///////////////////////////////////////////
  YawRateError = DesiredYawRate - imu_6500.GyroZ;
  InputYaw = .01*yaw_rate_control.pid_equation(YawRateError, deltaT_F);
  //////////////////////////////////////////////////////////////////////////////////////////////////////

  frontLeftMotorCommand  = (DesiredVerticalVelocity + InputPitch + InputRoll + InputYaw); 
  frontRightMotorCommand = (DesiredVerticalVelocity + InputPitch - InputRoll - InputYaw); 
  rearLeftMotorCommand   = (DesiredVerticalVelocity - InputPitch + InputRoll - InputYaw); 
  rearRightMotorCommand  = (DesiredVerticalVelocity - InputPitch - InputRoll + InputYaw);  

  if (RemoteControl.InputValue[Velocity] < ThrottleIdle)
  {
    frontLeftMotorCommand  = 0; 
    frontRightMotorCommand = 0;
    rearLeftMotorCommand   = 0; 
    rearRightMotorCommand  = 0;
    reset_pid();
  }
  
  dutyCycleFrontRight = constrain(frontRightMotorCommand*MIN_ON_TIME_MSEC_PWM + ON_TIME_RANGE_MSEC_PWM, MIN_ON_TIME_MSEC_PWM, MAX_ON_TIME_MSEC_PWM);
  dutyCycleFrontLeft  = constrain( frontLeftMotorCommand*MIN_ON_TIME_MSEC_PWM + ON_TIME_RANGE_MSEC_PWM, MIN_ON_TIME_MSEC_PWM, MAX_ON_TIME_MSEC_PWM);
  dutyCycleRearRight  = constrain( rearRightMotorCommand*MIN_ON_TIME_MSEC_PWM + ON_TIME_RANGE_MSEC_PWM, MIN_ON_TIME_MSEC_PWM, MAX_ON_TIME_MSEC_PWM);
  dutyCycleRearLeft   = constrain(  rearLeftMotorCommand*MIN_ON_TIME_MSEC_PWM + ON_TIME_RANGE_MSEC_PWM, MIN_ON_TIME_MSEC_PWM, MAX_ON_TIME_MSEC_PWM);

}


void reset_pid()
{
  pitch_control.pid_reset();
  pitch_rate_control.pid_reset();
  roll_control.pid_reset();
  roll_rate_control.pid_reset();
  yaw_rate_control.pid_reset();
}


void MadgwickConverge()
{
  for(int i = 0; i < 100000; i++)
  {
    imu_6500.getSensorData();
    Madgwick.Madgwick6DOF(imu_6500.GyroX_Filt, -imu_6500.GyroY_Filt, -imu_6500.GyroZ_Filt, -imu_6500.AccX_Filt, imu_6500.AccY_Filt, imu_6500.AccZ_Filt, .0005);
  }

  //loopRate(2000);
}

void motor_isr_ON()
{
  noInterrupts();
  turn_on_time = micros();
  quadcopter[MOTOR_FRONT_RIGHT/2-1].motor.turn_on();
  quadcopter[MOTOR_FRONT_LEFT/2-1].motor.turn_on();
  quadcopter[MOTOR_REAR_RIGHT/2-1].motor.turn_on();
  quadcopter[MOTOR_REAR_LEFT/2-1].motor.turn_on();

  quadcopter[MOTOR_FRONT_RIGHT/2-1].duration = dutyCycleFrontRight;
  quadcopter[MOTOR_FRONT_LEFT/2-1].duration = dutyCycleFrontLeft;
  quadcopter[MOTOR_REAR_RIGHT/2-1].duration = dutyCycleRearRight;
  quadcopter[MOTOR_REAR_LEFT/2-1].duration = dutyCycleRearLeft;

  motorTimer.begin(motor_isr_INTERMEDIATE, 120);
  interrupts();
}

void motor_isr_OFF()
{
  noInterrupts();
  turn_off_time = micros();
  if( ((turn_off_time - turn_on_time) >= quadcopter[MOTOR_FRONT_RIGHT/2-1].duration) && (quadcopter[MOTOR_FRONT_RIGHT/2-1].motor.flag == 1) )
  {
    quadcopter[MOTOR_FRONT_RIGHT/2-1].motor.turn_off();
  }
   if( ((turn_off_time - turn_on_time) >= quadcopter[MOTOR_FRONT_LEFT/2-1].duration) && (quadcopter[MOTOR_FRONT_LEFT/2-1].motor.flag == 1) )
  {
    quadcopter[MOTOR_FRONT_LEFT/2-1].motor.turn_off();
  }
  if( ((turn_off_time - turn_on_time) >= quadcopter[MOTOR_REAR_RIGHT/2-1].duration) && (quadcopter[MOTOR_REAR_RIGHT/2-1].motor.flag == 1) )
  {
    quadcopter[MOTOR_REAR_RIGHT/2-1].motor.turn_off();
  }
  if( ((turn_off_time - turn_on_time) >= quadcopter[MOTOR_REAR_LEFT/2-1].duration) && (quadcopter[MOTOR_REAR_LEFT/2-1].motor.flag == 1) )
  {
    quadcopter[MOTOR_REAR_LEFT/2-1].motor.turn_off();
  }     

  if((quadcopter[MOTOR_FRONT_RIGHT/2-1].motor.flag == 0) && \
     (quadcopter[MOTOR_FRONT_LEFT/2-1].motor.flag == 0)  && \
     (quadcopter[MOTOR_REAR_RIGHT/2-1].motor.flag == 0) && \
     (quadcopter[MOTOR_REAR_LEFT/2-1].motor.flag == 0) )
   {
      msec1Timer.end();
   }
  interrupts();
}

void motor_isr_INTERMEDIATE()
{
  motorTimer.end();
  msec1Timer.begin(motor_isr_OFF,1);
}
