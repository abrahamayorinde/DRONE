#ifndef FSI6_H
#define FSI6_H

#include "fsi6.h"
#include "fsi6_constants.h"

enum Input{Roll = 0, Pitch = 1, Velocity = 2, Yaw = 3};

class fsi6
{
    private:
        static const int validChannels = 5;
        static const int channelAmount = 6;
        static const unsigned long blankTime = 2100;
    public:
        float pitch_offset, roll_offset, yaw_offset;
        fsi6();
        static void isr();
        void calibrateRemote();
        static float InputValue[validChannels];

        //void fillArray();
};


#endif // FSI6_H
