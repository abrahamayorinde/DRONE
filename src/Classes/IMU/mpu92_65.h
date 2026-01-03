#ifndef MPU92_65_H
#define MPU92_65_H

#include "SPI.h"
#include "mpu6500.h"
#include "mpu92_65_constants.h"



class MPU_92_65
{
    private:
    bfs::Mpu6500 imu;
    public:
    MPU_92_65();
    void checkIMUSPISensor();
    void calibrateGyro();
    void getSensorData();
    int32_t gx_offset, gy_offset, gz_offset;
    int32_t GyroX_Filt, GyroY_Filt, GyroZ_Filt;
    int32_t GyroX, GyroY, GyroZ;
    int32_t AccX_Filt, AccY_Filt, AccZ_Filt;
    int32_t AccX, AccY, AccZ;
};



#endif // MPU92_65_H
