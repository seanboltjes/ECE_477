#ifndef IMU_9DOF_LSM6DS33_LIS3MDL_H
#define IMU_9DOF_LSM6DS33_LIS3MDL_H
#include <math.h>
#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include "../IMU_9DOF/IMU_9DOF.h"
#include "../IMU_6DOF_LSM6x/IMU_6DOF_LSM6x.h"

/*
Class for the  9-DOF IMU sensor
*/

/**
 * @brief The class to control the Adafruit  IMU module and any other  modules
 */
class IMU_9DOF_LSM6DS33_LIS3MDL : public IMU_9DOF
{    
public:
    virtual void Init();
    virtual void Init(Buzzer &buzzer);
    virtual ~IMU_9DOF_LSM6DS33_LIS3MDL();

    virtual bool GetMagnetometerVals(DirectionalValues& magn);
    virtual bool GetGyroVals(DirectionalValues& gyro);
    virtual bool GetAccelVals(DirectionalValues& accel);

protected:
    Adafruit_LIS3MDL* lis3mdl;
    LSM6* imu;
};
#endif