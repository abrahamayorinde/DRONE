#ifndef MOTORS_H
#define MOTORS_H
#include <Arduino.h>
#include "motors_constants.h"
class motor_obj
{
    private:
        int pin;         // Data member
    public:
        int flag = 0;
        void motor_io();
        void turn_on();
        void turn_off();
        motor_obj(int PIN);
};
/*ß
class motor_isr
{
    private:
    float motor_timer[4] = INFINITY;

    public:
    motor_isr();

}
*/
#endif //MOTORS_H
