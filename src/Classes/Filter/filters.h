
#ifndef FILTER_H
#define FILTER_H
#include <stdint.h>

class SecondOrderFilter
{
    public:
        float out_angle;// = 0;//initialize to accel angle upon start
        float gyro_filter;// = 0;//initialize to gyro value upon start
        float filter_x1;// = 0;
        float filter_x2;// = 0;
        float k;
        SecondOrderFilter();
        SecondOrderFilter(float Bandwidth);
        float Complementary(float accel_angle, float newRate,float timeDelta);
};


class FirstOrderFilter
{
    public:
        float tau;// =0.075;
        float compfilter_xangleC;// = 0;
        //float han[3] = {0,0,0};
        //float hanAvg = 0;
        float a;
        bool comp_start;// = false;
        
        FirstOrderFilter();
        FirstOrderFilter(float Tau);
        float Complementary(float newAngle, float newRate,float looptime);
};


class KalmanFilter
{
    public:
        double Q_angle = 0.1;//0.001;  // Process noise variance for the accelerometer
        double Q_bias = 0.01;//0.003;   // Process noise variance for the gyroscope bias
        double R_measure = 0.03; // Measurement noise variance
        double angle = 0, bias = 0, rate = 0; // Kalman filter state variables
        double P[2][2] = {{0, 0}, {0, 0}};    // Error covariance matrix
        double Kalman(double angle, double gyroRate, double accelAngle, float deltaT);
};


class MadgwickFilter
{
    private:
        float gain_Madgwick;
        float invSqrt(float x);
        float q[4];
    public:
        float yaw, pitch, roll;
        MadgwickFilter();
        MadgwickFilter(float filter_gain);
        void Madgwick9DOF(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float invSampleFreq);
        void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq);
        void getEuler(float &Pitch, float& Yaw, float &Roll);
        void getQuaternion(float& q, float& i, float& j, float& k);
};


class MahonyFilter
{
    private:
        float Kp;
        float Ki;
        float q[4];
    public:
        float yaw, pitch, roll;
        MahonyFilter();
        MahonyFilter(float Kp_Mahony, float Ki_Mahony);
        void Mahony_update(float ax, float ay, float az, float gx, float gy, float gz, float deltat);
        void getEuler(float& pitch, float& yaw, float& roll);
        void getQuaternion(float& q, float& i, float& j, float& k);
};


#endif // FILTER_H

