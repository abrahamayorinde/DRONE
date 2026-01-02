#ifndef MPU92_65_H
#define MPU92_65_H

#include "mpu92_65_constants.h"



void checkIMUSPISensor();
void calibrateGyro(int32_t& gx_offset, int32_t& gy_offset, int32_t& gz_offset);
void getSensorData();

#endif // MPU92_65_H
