#include "pid.h"
#include <stdlib.h>

PID::PID(float Throttle, float Proportional_gain, float Integral_gain, float Derivative_gain, float FeedForwardGainPitchRate, float Windup_Limit, float PID_Saturation, bool Clegg_Reset)
{
    this->throttle = Throttle;
    this->proportional_gain = Proportional_gain;
    this->integral_gain = Integral_gain;
    this->derivative_gain = Derivative_gain;
    this->feed_forward_derivative_gain = FeedForwardGainPitchRate;
    this->proportional_term = 0;
    this->integral_term = 0;
    this->derivative_term = 0;
    this->prior_error = 0;
    this->prior_integral_term = 0;
    this->prior_derivative_term = 0;
    this->integral_windup = Windup_Limit;
    this->pid_saturation = PID_Saturation;
    this->clegg_reset = Clegg_Reset;
}


float PID::pid_equation(float error, float deltaT)
{
    this->proportional_term = this->proportional_gain*error;
    
    this->integral_term = this->prior_integral_term + this->integral_gain*error*deltaT;
    

    if (this->integral_term > this->integral_windup)
    {
        this->integral_term = this->integral_windup;
    }
    else if (this->integral_term < -this->integral_windup)
    {
        this->integral_term = -this->integral_windup;
    }

    if(this->clegg_reset == true)
    {
        if( ( (this->prior_error < 0) && (error > 0) ) || ( (this->prior_error > 0) && (error < 0) ) )
        {
            this->integral_term = 0;
        }
    }
    
    if(this->throttle < 1060)
    {
        this->integral_term = 0;
    }
    
    this->derivative_term = (feed_forward_derivative_gain*derivative_gain*(error - prior_error)/deltaT +  (1 - feed_forward_derivative_gain)*prior_derivative_term);
    
    this->PIDOutput= this->proportional_term + this->integral_term + this->derivative_term;

    if (this->PIDOutput > this->pid_saturation)
    {
        this->PIDOutput = this->pid_saturation;
    }
    else if (this->PIDOutput < -this->pid_saturation)
    {
        this->PIDOutput = -this->pid_saturation;
    }
    
    this->prior_error = error;
    this->prior_integral_term = this->integral_term;
    this->prior_derivative_term = this->derivative_term;
    
    return this->PIDOutput;
}


void PID::pid_reset()
{
    this->proportional_term = 0;
    this->integral_term = 0;
    this->derivative_term = 0;
    this->prior_error = 0;
    this->prior_integral_term = 0;
    this->prior_derivative_term = 0;
}
