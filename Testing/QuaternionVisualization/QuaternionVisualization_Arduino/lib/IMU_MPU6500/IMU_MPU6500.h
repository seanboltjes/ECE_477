#ifndef IMU_MPU6500_H
#define IMU_MPU6500_H
#include <math.h>
#include <Wire.h>
#include <MPU6500_WE.h>
#include <xyzFloat.h>
#include "../DataStructures/DataStructures.h"


/**
 * @brief class for the MPU-6500 6 DoF IMU module
 */
class IMU_MPU6500 : public DataStructures
{    
public:
    // Getters
    virtual bool GetGyroVals(DirectionalValues& gyro);
    virtual bool GetAccelVals(DirectionalValues& accel);

    // Print Methods
    virtual void PrintReadingsGyro(DirectionalValues &gyro);
    virtual void PrintReadingsAccel(DirectionalValues &accel);
protected:
    virtual void InitIMU();
    virtual ~IMU_MPU6500();

    MPU6500_WE* imu;
};
#endif