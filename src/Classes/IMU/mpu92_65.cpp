#ifndef MPU92_65_C
#define MPU92_65_C


#include "mpu92_65.h"


MPU_92_65::MPU_92_65():imu(&SPI, 10){}

void MPU_92_65::checkIMUSPISensor()
{
    SPI.begin();
    // Initialize and configure IMU
    if (!imu.Begin())
    {
        Serial.println("Error initializing communication with IMU");
        while(1)
        {
        }
    }
    // Set the sample rate divider
    if (!imu.ConfigSrd(19))
    {
        Serial.println("Error configured SRD");
        while(1)
        {
        }
    }
}

void MPU_92_65::calibrateGyro()
{
  int32_t sum[3] = {0};

  for(int i = 0; i < IMU_CALIBRATION_COUNT; i++)
  {
    if (imu.Read())
    {
      sum[0] += imu.gyro_cnts_[0];
      sum[1] += imu.gyro_cnts_[1];
      sum[2] += imu.gyro_cnts_[2];
    }
    else
    {
      i--;
    }

    delay(1);
  }

  gx_offset = sum[0]/IMU_CALIBRATION_COUNT;
  gy_offset = sum[1]/IMU_CALIBRATION_COUNT;
  gz_offset = sum[2]/IMU_CALIBRATION_COUNT;
  
}


/*
 Get sensor data from the imu object.
 */
void MPU_92_65::getSensorData()
{
  if (imu.Read())
  {
    AccX = x_scale*(imu.accel_cnts_[0] - ax_offset);
    AccY = y_scale*(imu.accel_cnts_[1] - ay_offset);
    AccZ = z_scale*(imu.accel_cnts_[2] - az_offset);
  
    AccX_Filt = IMU_FILTER_ACCEL*AccX + (1-IMU_FILTER_ACCEL)*AccX_Filt;
    AccY_Filt = IMU_FILTER_ACCEL*AccY + (1-IMU_FILTER_ACCEL)*AccY_Filt;
    AccZ_Filt = IMU_FILTER_ACCEL*AccZ + (1-IMU_FILTER_ACCEL)*AccZ_Filt;

    GyroX = (imu.gyro_cnts_[0] - gx_offset)/GYRO_RESOLUTION;
    GyroY = (imu.gyro_cnts_[1] - gy_offset)/GYRO_RESOLUTION;
    GyroZ = (imu.gyro_cnts_[2] - gz_offset)/GYRO_RESOLUTION;

    GyroX_Filt = (IMU_FILTER_GYRO)*GyroX + (1-IMU_FILTER_GYRO)*GyroX_Filt;
    GyroY_Filt = (IMU_FILTER_GYRO)*GyroY + (1-IMU_FILTER_GYRO)*GyroY_Filt;
    GyroZ_Filt = (IMU_FILTER_GYRO)*GyroZ + (1-IMU_FILTER_GYRO)*GyroZ_Filt;
  }
}



#endif //MPU92_65_C
