#ifndef MOTORS_CPP
#define MOTORS_CPP
// Define the structure
#include "motors.h"
#include "motors_constants.h"
#include <stack>
/*
typedef struct motor_and_timeout
{
  uint64 timeout;
  motor_obj motor;
}

stack motor_and_timeout;
*/
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

/*
  motor_isr()
  {
    this->motor_timer;

    shortest = max(max(max(motor_timer[0], motor_timer[1]), motor_timer[2]), motor_timer[4]);

    motor_and_timeout.push()
    //Timer1.initialize(shortest); 
    timer

  }


  */
#endif //MOTORS_CPP
