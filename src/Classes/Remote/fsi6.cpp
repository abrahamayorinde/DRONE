
#include <Arduino.h>
#include "fsi6.h"
#include "fsi6_constants.h"
#include <stdlib.h>


fsi6::fsi6()
{
}

float fsi6::InputValue[validChannels] = {0,0,0,0,0};

void fsi6::isr()
{
    static unsigned long microsAtLastPulse = 0;
    static unsigned long time = 0;
    static unsigned long previousMicros = 0;
    static unsigned long pulseCounter = 0;

    microsAtLastPulse = micros();
    time = microsAtLastPulse - previousMicros;
    previousMicros = microsAtLastPulse;

    if (time > blankTime)
    {
        // Blank detected: restart from channel 1
        pulseCounter = 0;
    }
    else
    {
        // Store times between pulses as channel values
        if (pulseCounter < channelAmount)
        {
            InputValue[pulseCounter] = time;
            ++pulseCounter;
        }
    }
}

void fsi6::calibrateRemote()
{
  for(int i = 0; i<REMOTE_CALIBRATION_COUNT; i++)
  {
    yaw_offset += InputValue[Yaw];
    pitch_offset += InputValue[Pitch];
    roll_offset += InputValue[Roll];
  }

  yaw_offset /= REMOTE_CALIBRATION_COUNT;
  pitch_offset /= REMOTE_CALIBRATION_COUNT;
  roll_offset /= REMOTE_CALIBRATION_COUNT;
}

