#include <Wire.h>
#include <stdint.h>
#include "droneCFG.h"

#define MICROSEC_PER_SECOND 1000000

///////////////////////////////////  EXTERN    VARIABLES      ///////////////////////////////////

extern int32_t AccX, AccY, AccZ;
extern int32_t GyroX, GyroY, GyroZ;
extern int32_t GyroX_Filt, GyroY_Filt, GyroZ_Filt;
extern int32_t AccX_Filt, AccY_Filt, AccZ_Filt;
extern int32_t gx_offset, gy_offset, gz_offset;



///////////////////////////////////   LOCAL   VARIABLES      ///////////////////////////////////

float deltaT_F = 0;
unsigned long startTime = 0;
unsigned long endTime = 0;

int32_t InputThrottle = 0;

float DesiredRollAngle, DesiredPitchAngle, DesiredRollRate, DesiredPitchRate, DesiredYawRate, DesiredVerticalVelocity = {0};
float DesiredRollRateInput, DesiredPitchRateInput = {0};


float RollAngleError,  RollRateError  = {0};
float PitchAngleError, PitchRateError = {0};

float YawRateError = {0};

float InputPitch, InputRoll, InputYaw = {0};

float dutyCycleRearLeft, dutyCycleFrontLeft, dutyCycleRearRight, dutyCycleFrontRight = {0};
float rearLeftMotorCommand, frontLeftMotorCommand, rearRightMotorCommand, frontRightMotorCommand = {0};
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////      CONSTANTS      ///////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////      FUNCTION PROTOTYPES      //////////////////////////////

////////////////////////////////////      OBJECTS      ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

void setup() 
{
  Serial.begin(115200);

  checkIMUSPISensor();

  Serial.print("Setup starting!");
  pinMode(MOTOR_FRONT_RIGHT, OUTPUT);
  pinMode(MOTOR_FRONT_LEFT, OUTPUT);
  pinMode(MOTOR_REAR_RIGHT, OUTPUT);
  pinMode(MOTOR_REAR_LEFT, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(interruptPin), RemoteControl.isr, RISING);

  analogWriteResolution(8); //Resolution of data written to pinMode outputs is 8-bit

  calibrateGyro(gx_offset, gy_offset, gz_offset);

  RemoteControl.calibrateRemote();  

  getSensorData();
  
  MadgwickConverge();
}

void loop() 
{
  startTime = endTime;
  endTime = micros();
  deltaT_F = ((float)(endTime - startTime)/MICROSEC_PER_SECOND); //convert from microseconds to seconds

  getSensorData();
  
  DesiredRollAngle  =  (RemoteControl.InputValue[Roll] - RemoteControl.roll_offset)/ROLL_MARGIN;
  DesiredRollAngle  = maxRoll*constrain(DesiredRollAngle, -1, 1);

  DesiredPitchAngle = -(RemoteControl.InputValue[Pitch] - RemoteControl.pitch_offset)/PITCH_MARGIN;//-1 coeff pitch angle correponds to imu orientation
  DesiredPitchAngle = maxPitch*constrain(DesiredPitchAngle, -1, 1);

  DesiredYawRate = -(RemoteControl.InputValue[Yaw] - RemoteControl.yaw_offset )/YAW_MARGIN;//-1 coeff pitch angle correponds to imu orientation
  DesiredYawRate = maxYaw*constrain(DesiredYawRate, -1, 1);

  DesiredVerticalVelocity = (RemoteControl.InputValue[Velocity] - THROTTLE_RANGE)/THROTTLE_RANGE;      
  DesiredVerticalVelocity = constrain(DesiredVerticalVelocity, 0, 1);

  Madgwick.Madgwick6DOF(GyroX_Filt, -GyroY_Filt, -GyroZ_Filt, -AccX_Filt, AccY_Filt, AccZ_Filt,  deltaT_F);


  ////////////////////////////////////////////// ROLL  CONTROL /////////////////////////////////////////
  RollAngleError = DesiredRollAngle - ( Madgwick.roll);
  DesiredRollRate = roll_control.pid_equation(RollAngleError, deltaT_F);

  //Serial.println("RollAngleError:");Serial.println(RollAngleError);

  DesiredRollRate = constrain(DesiredRollRate, ROLL_RATE_MIN, ROLL_RATE_MAX);
  DesiredRollRateInput = (1 - Roll_Rate_Damping)*DesiredRollRateInput + Roll_Rate_Damping * DesiredRollRate;

  //Serial.println("DesiredRollRateInput:");Serial.println(DesiredRollRateInput);

  RollRateError = DesiredRollRateInput - GyroX;
  InputRoll = .01*roll_rate_control.pid_equation(RollRateError, deltaT_F);
  ////////////////////////////////////////////// ROLL  CONTROL /////////////////////////////////////////


  ///////////////////////////////////////////// PITCH  CONTROL /////////////////////////////////////////
  PitchAngleError = DesiredPitchAngle - (-Madgwick.pitch + 9.5);// ;
  DesiredPitchRate = pitch_control.pid_equation(PitchAngleError, deltaT_F);

  DesiredPitchRate = constrain(DesiredPitchRate, PITCH_RATE_MIN, PITCH_RATE_MAX);
  DesiredPitchRateInput = (1 - Pitch_Rate_Damping)*DesiredPitchRateInput + Pitch_Rate_Damping * DesiredPitchRate;

  PitchRateError = (DesiredPitchRateInput + GyroY);
  InputPitch = .01* pitch_rate_control.pid_equation(PitchRateError, deltaT_F);
  ///////////////////////////////////////////// PITCH  CONTROL /////////////////////////////////////////


  ///////////////////////////////////////////// YAW  CONTROL /////////////////////////////////////////
  YawRateError = DesiredYawRate - GyroZ;
  InputYaw = .01*yaw_rate_control.pid_equation(YawRateError, deltaT_F);
  ///////////////////////////////////////////// YAW  CONTROL /////////////////////////////////////////


  frontLeftMotorCommand  = (DesiredVerticalVelocity + InputPitch + InputRoll + InputYaw); 
  frontRightMotorCommand = (DesiredVerticalVelocity + InputPitch - InputRoll - InputYaw); 
  rearLeftMotorCommand   = (DesiredVerticalVelocity - InputPitch + InputRoll - InputYaw); 
  rearRightMotorCommand  = (DesiredVerticalVelocity - InputPitch - InputRoll + InputYaw);  

  Serial.print("frontLeftMotorCommand:");Serial.println(frontLeftMotorCommand);
  Serial.print("frontRightMotorCommand:");Serial.println(frontRightMotorCommand);
  Serial.print("rearLeftMotorCommand:");Serial.println(rearLeftMotorCommand);
  Serial.print("rearRightMotorCommand:");Serial.println(rearRightMotorCommand);


  if (RemoteControl.InputValue[Velocity] < ThrottleIdle)
  {
    frontLeftMotorCommand  = 0; 
    frontRightMotorCommand = 0;
    rearLeftMotorCommand   = 0; 
    rearRightMotorCommand  = 0;
    reset_pid();
  }
  
  dutyCycleRearLeft   = constrain(  rearLeftMotorCommand*MIN_ON_TIME_MSEC_PWM + ON_TIME_RANGE_MSEC_PWM, MIN_ON_TIME_MSEC_PWM, MAX_ON_TIME_MSEC_PWM);
  dutyCycleFrontLeft  = constrain( frontLeftMotorCommand*MIN_ON_TIME_MSEC_PWM + ON_TIME_RANGE_MSEC_PWM, MIN_ON_TIME_MSEC_PWM, MAX_ON_TIME_MSEC_PWM);
  dutyCycleRearRight  = constrain( rearRightMotorCommand*MIN_ON_TIME_MSEC_PWM + ON_TIME_RANGE_MSEC_PWM, MIN_ON_TIME_MSEC_PWM, MAX_ON_TIME_MSEC_PWM);
  dutyCycleFrontRight = constrain(frontRightMotorCommand*MIN_ON_TIME_MSEC_PWM + ON_TIME_RANGE_MSEC_PWM, MIN_ON_TIME_MSEC_PWM, MAX_ON_TIME_MSEC_PWM);

  Serial.print("dutyCycleRearLeft:");Serial.println(dutyCycleRearLeft  );
  Serial.print("dutyCycleFrontLeft:");Serial.println(dutyCycleFrontLeft );
  Serial.print("dutyCycleRearRight:");Serial.println(dutyCycleRearRight );
  Serial.print("dutyCycleFrontRight:");Serial.println(dutyCycleFrontRight);
  
  int flagFR = 0;
  int flagFL = 0;
  int flagRR = 0;
  int flagRL = 0;
  int wentLow = 0;
  int pulseStart, timer = {0};

  while(micros() - startTime < 250)// [250 for 2000Hz], [83 for 3000Hz], [0 for 4000Hz]
  {
  }

  if(micros() - startTime >= 250)// [250 for 2000Hz], [83 for 3000Hz], [0 for 4000Hz]
  {  
    pulseStart = micros();
    digitalWriteFast(MOTOR_FRONT_RIGHT, HIGH);
    digitalWriteFast(MOTOR_FRONT_LEFT, HIGH);
    digitalWriteFast(MOTOR_REAR_RIGHT, HIGH);
    digitalWriteFast(MOTOR_REAR_LEFT , HIGH);
    while (wentLow < 4 ) 
    { //Keep going until final (6th) pulse is finished, then done
      timer = micros();
      if ((dutyCycleRearLeft <= (timer - pulseStart)) && (flagRL==0)) 
      {
        digitalWriteFast(MOTOR_REAR_LEFT, LOW);
        wentLow = wentLow + 1;
        flagRL = 1;
      }             
      if ((dutyCycleFrontRight <= (timer - pulseStart)) && (flagFR==0)) 
      {
        digitalWriteFast(MOTOR_FRONT_RIGHT, LOW);
        wentLow = wentLow + 1;
        flagFR = 1;
      }
      if ((dutyCycleFrontLeft <= (timer - pulseStart)) && (flagFL==0)) 
      {
        digitalWriteFast(MOTOR_FRONT_LEFT, LOW);
        wentLow = wentLow + 1;
        flagFL = 1;
      }
      if ((dutyCycleRearRight <= (timer - pulseStart)) && (flagRR==0)) 
      {
        digitalWriteFast(MOTOR_REAR_RIGHT, LOW);
        wentLow = wentLow + 1;
        flagRR = 1;
      }
    }
  }

  loopRate(2000);
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
  for(int i = 0; i < 10000; i++)
  {
    Madgwick.Madgwick6DOF(GyroX_Filt, -GyroY_Filt, -GyroZ_Filt, -AccX_Filt, AccY_Filt, AccZ_Filt, .0005);
  }
  loopRate(2000);
}


void loopRate(int freq) 
{
  //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
  /*
   * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
   * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until 
   * the correct time has passed since the start of the current loop for the desired loop rate in Hz. 2kHz is a good rate to 
   * be at because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us have a little room to add extra computations
   * and remain above 2kHz, without needing to retune all of our filtering parameters.
   */
  float invFreq = (1.0/freq)*MICROSEC_PER_SECOND;
  uint32_t current_time = micros();
  uint32_t checker = 0;
  
  //Sit in loop until appropriate time has passed
  while (invFreq > (float)(checker - current_time)) 
  {
    checker = micros();
  }
}