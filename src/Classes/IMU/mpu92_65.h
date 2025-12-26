#ifndef MPU92_65_H
#define MPU92_65_H

#include "mpu92_65_constants.h"



void checkIMUSPISensor();
void calibrateGyro(float& gx_offset, float& gy_offset, float& gz_offset);
void getSensorData();

#endif // MPU92_65_H
