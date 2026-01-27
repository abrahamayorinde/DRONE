#ifndef FSI6_H
#define FSI6_H

#include "fsi6.h"
#include "fsi6_constants.h"

enum Input{Roll = 0, Pitch = 1, Velocity = 2, Yaw = 3};

class fsi6
{
    private:
        static const int8_t validChannels = 5;
        static const int8_t channelAmount = 6;
        static const uint32_t blankTime = 2100;
    public:
        float pitch_offset, roll_offset, yaw_offset = {0};
        fsi6();
        static void isr();
        void calibrateRemote();
        static float InputValue[validChannels];
};


#endif // FSI6_H
