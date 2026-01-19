#ifndef MOTORS_CPP
#define MOTORS_CPP
// Define the structure
#include "motors.h"
#include "motors_constants.h"

    motor_obj::motor_obj(int PIN):pin(PIN)
    {
        motor_io();
    }
    
    void motor_obj::motor_io()
    {
        pinMode(pin, OUTPUT);
    }
    
    void motor_obj::turn_on()
    {
        digitalWriteFast(pin, HIGH);
        flag = 1;
    }
    
    void motor_obj::turn_off()
    {
        digitalWriteFast(pin, LOW);
        flag = 0;
    }


#endif //MOTORS_CPP
