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

void setup() 
{
  Serial.begin(115200);

  imu_6500.checkIMUSPISensor();//verify that the sensor is working correctly.  If it is not, the software will not halt here.

  Serial.println("Sensor check passed.");

  attachInterrupt(digitalPinToInterrupt(interruptPin), RemoteControl.isr, RISING);

  Serial.println("Interrupt attach complete.");

  analogWriteResolution(8); //Resolution of data written to pinMode outputs is 8-bit

  Serial.println("Write resolution (8) complete.");

  imu_6500.calibrateGyro();

  Serial.println("Gyro offsets calculated.");

  RemoteControl.calibrateRemote();  //Calculate offsets with yaw, pitch and roll at center position and throttle at lowest position.

  Serial.println("Remote offsets calculated.");

  MadgwickConverge(); //allow Madgwick filter to converge before utilizing the data in the control algorithm

  Serial.println("Madgwick filter converged to stable value.");
}

void loop() 
{
  previousTime = currentStart;
  currentStart = micros();
  deltaT_F = ((float)(currentStart - previousTime)/MICROSEC_PER_SECOND); //convert from microseconds to seconds

  imu_6500.getSensorData();

  DesiredRollAngle  =  (RemoteControl.InputValue[Roll] - RemoteControl.roll_offset)/ROLL_MARGIN;
  Serial.print("DesiredRollAngle:");Serial.println(DesiredRollAngle);
  DesiredRollAngle  = maxRoll*constrain(DesiredRollAngle, -1, 1);
  Serial.print("DesiredRollAngle_shrunk:");Serial.println(DesiredRollAngle);

  DesiredPitchAngle = -(RemoteControl.InputValue[Pitch] - RemoteControl.pitch_offset)/PITCH_MARGIN;//-1 coeff pitch angle correponds to imu orientation
  DesiredPitchAngle = maxPitch*constrain(DesiredPitchAngle, -1, 1);

  DesiredYawRate = -(RemoteControl.InputValue[Yaw] - RemoteControl.yaw_offset )/YAW_MARGIN;//-1 coeff pitch angle correponds to imu orientation
  DesiredYawRate = maxYaw*constrain(DesiredYawRate, -1, 1);

  DesiredVerticalVelocity = (RemoteControl.InputValue[Velocity] - THROTTLE_RANGE)/THROTTLE_RANGE;      
  DesiredVerticalVelocity = constrain(DesiredVerticalVelocity, 0, 1);

  Madgwick.Madgwick6DOF(imu_6500.GyroX_Filt, -imu_6500.GyroY_Filt, -imu_6500.GyroZ_Filt, -imu_6500.AccX_Filt, imu_6500.AccY_Filt, imu_6500.AccZ_Filt,  deltaT_F);


  ////////////////////////////////////////////// ROLL  CONTROL /////////////////////////////////////////
  Serial.print("Roll_Angle:");Serial.println(Madgwick.roll);
  RollAngleError = DesiredRollAngle - ( Madgwick.roll);
  Serial.print("RollAngleError:");Serial.println(RollAngleError);
  DesiredRollRate = roll_control.pid_equation(RollAngleError, deltaT_F);
  Serial.print("DesiredRollRate:");Serial.println(DesiredRollRate);

  DesiredRollRate = constrain(30*DesiredRollRate, ROLL_RATE_MIN, ROLL_RATE_MAX);
  //DesiredRollRate = constrain(DesiredRollRate, ROLL_RATE_MIN, ROLL_RATE_MAX);
  DesiredRollRateInput = (1 - Roll_Rate_Damping)*DesiredRollRateInput + Roll_Rate_Damping * DesiredRollRate;
  Serial.print("DesiredRollRateInput:");Serial.println(DesiredRollRateInput);

  RollRateError = DesiredRollRateInput - imu_6500.GyroX;
  InputRoll = .01*roll_rate_control.pid_equation(RollRateError, deltaT_F);
  Serial.print("InputRoll:");Serial.println(InputRoll);

  //////////////////////////////////////////////////////////////////////////////////////////////////////


  ///////////////////////////////////////////// PITCH  CONTROL /////////////////////////////////////////
  //Serial.print("Pitch_Angle:");Serial.println(Madgwick.pitch + 9.5);
  PitchAngleError = DesiredPitchAngle - ((-Madgwick.pitch) + 9.5);// ;
  DesiredPitchRate = pitch_control.pid_equation(PitchAngleError, deltaT_F);

  DesiredPitchRate = constrain(30*DesiredPitchRate, PITCH_RATE_MIN, PITCH_RATE_MAX);
  //DesiredPitchRate = constrain(DesiredPitchRate, PITCH_RATE_MIN, PITCH_RATE_MAX);
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
  
  dutyCycleRearLeft   = constrain(  rearLeftMotorCommand*MIN_ON_TIME_MSEC_PWM + ON_TIME_RANGE_MSEC_PWM, MIN_ON_TIME_MSEC_PWM, MAX_ON_TIME_MSEC_PWM);
  dutyCycleFrontLeft  = constrain( frontLeftMotorCommand*MIN_ON_TIME_MSEC_PWM + ON_TIME_RANGE_MSEC_PWM, MIN_ON_TIME_MSEC_PWM, MAX_ON_TIME_MSEC_PWM);
  dutyCycleRearRight  = constrain( rearRightMotorCommand*MIN_ON_TIME_MSEC_PWM + ON_TIME_RANGE_MSEC_PWM, MIN_ON_TIME_MSEC_PWM, MAX_ON_TIME_MSEC_PWM);
  dutyCycleFrontRight = constrain(frontRightMotorCommand*MIN_ON_TIME_MSEC_PWM + ON_TIME_RANGE_MSEC_PWM, MIN_ON_TIME_MSEC_PWM, MAX_ON_TIME_MSEC_PWM);

  int disabled = 0;
  int pulseStart, timer = {0};


  while(micros() - previousTime < 250)// [250 for 2000Hz], [83 for 3000Hz], [0 for 4000Hz]
  {
  }

  if(micros() - previousTime >= 250)  // [250 for 2000Hz], [83 for 3000Hz], [0 for 4000Hz]
  {  
    pulseStart = micros();

    front_right.turn_on();
    front_left.turn_on();
    rear_right.turn_on();
    rear_left.turn_on();

    while (disabled < 4 ) 
    { //Keep going until all motors high time is completed 
      timer = micros();

      if((dutyCycleRearLeft <= (timer-pulseStart)) && rear_left.flag == 1)
      {
        rear_left.turn_off();
        disabled = disabled + 1;
      }             
      if((dutyCycleFrontRight <= (timer-pulseStart)) && front_right.flag == 1)
      {
        front_right.turn_off();
        disabled = disabled + 1;
      }
      if((dutyCycleFrontLeft <= (timer-pulseStart)) && front_left.flag == 1)
      {
        front_left.turn_off();
        disabled = disabled + 1;
      }
      if((dutyCycleRearRight <= (timer-pulseStart)) && rear_right.flag == 1)
      {
        rear_right.turn_off();
        disabled = disabled + 1;
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
  for(int i = 0; i < 100000; i++)
  {
    imu_6500.getSensorData();
    Madgwick.Madgwick6DOF(imu_6500.GyroX_Filt, -imu_6500.GyroY_Filt, -imu_6500.GyroZ_Filt, -imu_6500.AccX_Filt, imu_6500.AccY_Filt, imu_6500.AccZ_Filt, .0005);
  }
  loopRate(2000);
}


void loopRate(int loopfreq) 
{
  //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
  /*
   * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
   * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until 
   * the correct time has passed since the start of the current loop for the desired loop rate in Hz. 2kHz is a good rate to 
   * be at because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us have a little room to add extra computations
   * and remain above 2kHz, without needing to retune all of our filtering parameters.
   */
  float loopDuration = (1.0/loopfreq)*MICROSEC_PER_SECOND;
  //uint32_t current_time = micros();
  uint32_t checker = micros();
  
  //Sit in loop until appropriate time has passed
  //while (invFreq > (float)(checker - current_time)) 
  while ((float)(checker - currentStart) < loopDuration ) 
  {
    checker = micros();
  }
}