#ifndef IMU_3DOF_H
#define IMU_3DOF_H
#include <Wire.h>
#include <Adafruit_LSM303.h>
#include "../IMU_6DOF/IMU_6DOF.h"


class IMU_3DOF_MAGNETOMETER
{    
public:
    virtual void Init();
    virtual ~IMU_3DOF_MAGNETOMETER();

    virtual bool GetMagnetometerVals(IMU_6DOF::DirectionalValues& gyro);

protected:
    Adafruit_LSM303* lsm;
};
#endif