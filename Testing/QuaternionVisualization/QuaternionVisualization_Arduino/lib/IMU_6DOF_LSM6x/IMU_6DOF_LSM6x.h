#ifndef IMU_6DOF_LSM6X_H
#define IMU_6DOF_LSM6X_H
#include <LSM6.h>
#include <Wire.h>
#include "../IMU_6DOF/IMU_6DOF.h"

/*
Class for the LSM6DS33 6-DOF IMU sensor 
Can be adapted for the 9-DOF LSM6DS33 + LIS3MDL IMU
*/

/**
 * @brief The class to control the Adafruit LSM6DS33 IMU module and any other LSM6xxx modules
 */
class IMU_6DOF_LSM6x : public IMU_6DOF
{    
public:
    virtual void Init();
    virtual ~IMU_6DOF_LSM6x();

    virtual bool GetGyroVals(DirectionalValues& gyro);
    virtual bool GetAccelVals(DirectionalValues& accel);

protected:
    LSM6* imu;
};
#endif