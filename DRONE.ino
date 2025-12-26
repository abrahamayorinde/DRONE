#include <Wire.h>
#include <stdint.h>
#include "droneCFG.h"

//bfs::Mpu6500 imu(&SPI, 10);


///////////////////////////////////      VARIABLES      ///////////////////////////////////

extern float AccX, AccY, AccZ;
extern float GyroX, GyroY, GyroZ;
extern float GyroX_Filt, GyroY_Filt, GyroZ_Filt;
extern float AccX_Filt, AccY_Filt, AccZ_Filt;
extern float gx_offset, gy_offset, gz_offset;


//float Madgw_roll, Madgw_pitch, Madgw_yaw = 0;
//float Mahony_pitch, Mahony_roll, Mahony_yaw = {0};

unsigned long deltaT = 0;
float deltaT_F = 0;
unsigned long startTime = 0;
unsigned long endTime = 0;

float InputThrottle;

float DesiredRollAngle, DesiredPitchAngle, DesiredRollRate, DesiredPitchRate, DesiredYawRate, DesiredVerticalVelocity = {0};
float DesiredRollRateInput, DesiredPitchRateInput = 0;
float remoteRoll_Lpf, remotePitch_Lpf, remoteYaw_Lpf, remoteThrottle_Lpf = {1.0};

float dutyCycleRearLeft, dutyCycleFrontLeft, dutyCycleRearRight, dutyCycleFrontRight = {0};
float rearLeftMotorCommand, frontLeftMotorCommand, rearRightMotorCommand, frontRightMotorCommand;

float RollAngleError,  RollRateError,  RollAngleError_F,  RollRateError_F  = {0};
float PitchAngleError, PitchRateError, PitchAngleError_F, PitchRateError_F = {0};

float YawRateError = {0};

float InputPitch, InputRoll, InputYaw = {0};

float remote_yaw_offset = 0;  //1512;
float remote_pitch_offset = 0;//1452;//1507;//1452
float remote_roll_offset = 0; //1500;//1457;

uint16_t loopcounter = 0;
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////      CONSTANTS      ///////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////      FUNCTION PROTOTYPES      //////////////////////////////
void getSensorData();

////////////////////////////////////      OBJECTS      ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

void setup() 
{
  Serial.begin(115200);

  checkIMUSPISensor();

  pinMode(MOTOR_FRONT_RIGHT, OUTPUT);
  pinMode(MOTOR_FRONT_LEFT, OUTPUT);
  pinMode(MOTOR_REAR_RIGHT, OUTPUT);
  pinMode(MOTOR_REAR_LEFT, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(interruptPin), RemoteControl.isr, RISING);

  analogWriteResolution(8);

  calibrateGyro(gx_offset, gy_offset, gz_offset);

  RemoteControl.calibrateRemote();  

  getSensorData();
  
  MadgwickConverge();
}

void loop() 
{
  startTime = endTime;
  endTime = micros();
  deltaT = endTime - startTime;

  deltaT_F = ((float)deltaT/1000000.0); 

  getSensorData();
  
  //DesiredRollAngle  =  ( (1-remoteRoll_Lpf)*DesiredRollAngle + remoteRoll_Lpf*(InputValue[Roll]  - roll_offset) )/500;
  DesiredRollAngle  =  (RemoteControl.InputValue[Roll]  - RemoteControl.roll_offset)/500;
  DesiredRollAngle  = maxRoll*constrain(DesiredRollAngle, -1, 1);

  //DesiredPitchAngle = -( (1-remotePitch_Lpf)*DesiredPitchAngle + remotePitch_Lpf*(InputValue[Pitch]  - pitch_offset) )/500;//-1 coeff pitch angle correponds to imu orientation
  DesiredPitchAngle = -(RemoteControl.InputValue[Pitch]  - RemoteControl.pitch_offset)/500;//-1 coeff pitch angle correponds to imu orientation
  DesiredPitchAngle = maxPitch*constrain(DesiredPitchAngle, -1, 1);

  //DesiredYawRate = -( (1-remoteYaw_Lpf)*DesiredYawRate + remoteYaw_Lpf*(InputValue[Yaw]  - yaw_offset) )/500;//-1 coeff pitch angle correponds to imu orientation
  DesiredYawRate = -(RemoteControl.InputValue[Yaw] - RemoteControl.yaw_offset )/500;//-1 coeff pitch angle correponds to imu orientation
  DesiredYawRate = maxYaw*constrain(DesiredYawRate, -1, 1);

  DesiredVerticalVelocity = (RemoteControl.InputValue[Velocity] - 1000)/1000;     
  DesiredVerticalVelocity = constrain(DesiredVerticalVelocity, 0, 1);

  Madgwick.Madgwick6DOF(GyroX_Filt, -GyroY_Filt, -GyroZ_Filt, -AccX_Filt, AccY_Filt, AccZ_Filt,  deltaT_F);


  ////////////////////////////////////////////// ROLL  CONTROL /////////////////////////////////////////
  RollAngleError = DesiredRollAngle - ( Madgwick.roll);
  DesiredRollRate = roll_control.pid_equation(RollAngleError, deltaT_F);

  Serial.println("RollAngleError:");Serial.println(RollAngleError);

  DesiredRollRate = constrain(DesiredRollRate*30, -240, 240);
  DesiredRollRateInput = (1 - Roll_Rate_Damping)*DesiredRollRateInput + Roll_Rate_Damping * DesiredRollRate;

  Serial.println("DesiredRollRateInput:");Serial.println(DesiredRollRateInput);

  RollRateError = DesiredRollRateInput - GyroX;
  InputRoll = .01*roll_rate_control.pid_equation(RollRateError, deltaT_F);
  ////////////////////////////////////////////// ROLL  CONTROL /////////////////////////////////////////


  ///////////////////////////////////////////// PITCH  CONTROL /////////////////////////////////////////
  PitchAngleError = DesiredPitchAngle - (-Madgwick.pitch + 9.5);// ;
  DesiredPitchRate = pitch_control.pid_equation(PitchAngleError, deltaT_F);

  DesiredPitchRate = constrain(30*DesiredPitchRate, -240, 240);
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

  
  if (RemoteControl.InputValue[Velocity] < ThrottleIdle)
  {
    frontLeftMotorCommand  = 0; 
    frontRightMotorCommand = 0;
    rearLeftMotorCommand   = 0; 
    rearRightMotorCommand  = 0;
    reset_pid();
  }
  
  dutyCycleRearLeft   = constrain(  rearLeftMotorCommand*125 + 125, 125, 250);
  dutyCycleFrontLeft  = constrain( frontLeftMotorCommand*125 + 125, 125, 250);
  dutyCycleRearRight  = constrain( rearRightMotorCommand*125 + 125, 125, 250);
  dutyCycleFrontRight = constrain(frontRightMotorCommand*125 + 125, 125, 250);


  int flagFR = 0;
  int flagFL = 0;
  int flagRR = 0;
  int flagRL = 0;
  int wentLow = 0;
  int pulseStart, timer = {0};

  while(micros() - startTime <250)// [250 for 2000Hz], [375 for 2000Hz], [125 for 4000Hz]
  {
  }

  if(micros() - startTime >= 250)// [250 for 2000Hz], [375 for 2000Hz], [125 for 4000Hz]
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
  float invFreq = (1.0/freq)*1000000.0;
  unsigned long current_time = micros();
  unsigned long checker = 0;
  
  //Sit in loop until appropriate time has passed
  while (invFreq > (checker - current_time)) 
  {
    checker = micros();
  }
}