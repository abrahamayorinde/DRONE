#ifndef PID_H
#define PID_H

#include "pid_constants.h"

class PID
{
    public:
    float throttle;
    
    float proportional_gain;
    float integral_gain;
    float derivative_gain;
    
    double proportional_term;
    double integral_term;
    double derivative_term;
    
    double prior_error;
    double prior_integral_term;
    double prior_derivative_term;
    
    //Feed-forward terms
    float feed_forward_derivative_gain;
    
    //Saturation
    float integral_windup;
    float pid_saturation;
    
    bool clegg_reset;
    
    float PIDOutput;
    float pid_equation(float error, float deltaT);
    void pid_reset();
    
    PID(float Throttle, float Proportional_gain, float Integral_gain, float Derivative_gain, float FeedforwardDerivative_gain, float Windup_Limit, float PID_Saturation, bool Clegg_Reset);
};


#endif // PID_H
