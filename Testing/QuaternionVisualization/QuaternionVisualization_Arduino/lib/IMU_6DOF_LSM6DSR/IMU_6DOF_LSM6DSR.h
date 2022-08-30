#ifndef IMU_6DOF_LSM6DSR_H
#define IMU_6DOF_LSM6DSR_H
#include <LSM6DSRSensor.h>
#include <Wire.h>
#include "../IMU_6DOF/IMU_6DOF.h"
#include "../Buzzer/Buzzer.h"

/*
Class for the LSM6DSR 6-DOF IMU sensor 
*/

/**
 * @brief The class to control the Adafruit LSM6DSR IMU module
 */
class IMU_6DOF_LSM6DSR : public IMU_6DOF
{    
public:
    virtual void Init();
    virtual void Init(Buzzer &buzzer);
    virtual ~IMU_6DOF_LSM6DSR();

    virtual bool GetGyroVals(DirectionalValues& gyro);
    virtual bool GetAccelVals(DirectionalValues& accel);

protected:
    LSM6DSRSensor* imu;
};
#endif