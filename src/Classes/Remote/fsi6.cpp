
#include <Arduino.h>
#include "fsi6.h"
#include "fsi6_constants.h"
#include <stdlib.h>


fsi6::fsi6()
{
}

float_t fsi6::InputValue[validChannels] = {0,0,0,0,0};

/*
 Interrupt service routine to process the channel data for the roll, pitch, yaw and velocity inputs.
 Channel 0 -> roll
 Channel 1 -> pitch
 Channel 2 -> yaw
 Channel 3 -> velocity
 */
void fsi6::isr()
{
  noInterrupts();
    static uint32_t microsAtLastPulse = 0;
    static uint32_t time = 0;
    static uint32_t previousMicros = 0;
    static uint32_t pulseCounter = 0;

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
    interrupts();
}


/*
 At resting position the remote control values may not be at their default values.
 The default remote control value for the yaw, pitch and roll commands should be in the center of the allowable range {1000,2000} at 1500.
 However, in reality, the analog remote control may have some slack where the actual values may deviate from the default value +/- several counts.
 This routine calculates the actual default value.  This default value is subsequently used as the offset to compensate for the remote control inputs.
 */
void fsi6::calibrateRemote()
{
  for(uint16_t i = 0; i<REMOTE_CALIBRATION_COUNT; i++)
  {
    yaw_offset += InputValue[Yaw];
    pitch_offset += InputValue[Pitch];
    roll_offset += InputValue[Roll];
  }

  yaw_offset /= REMOTE_CALIBRATION_COUNT;
  pitch_offset /= REMOTE_CALIBRATION_COUNT;
  roll_offset /= REMOTE_CALIBRATION_COUNT;
}

