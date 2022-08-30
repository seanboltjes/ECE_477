#ifndef IMU_9DOF_9250_H
#define IMU_9DOF_9250_H
#include <math.h>
#include <Wire.h>
#include "../IMU_9DOF/IMU_9DOF.h"
#include "../Buzzer/Buzzer.h"
#include "MPU6500_WE.h"
#include "xyzFloat.h"


/**
 * @brief 
 */
class IMU_9DOF_9250 : public IMU_9DOF
{    
public:
    virtual void Init();
    virtual void Init(Buzzer &buzzer);
    virtual ~IMU_9DOF_9250();

    virtual bool GetMagnetometerVals(DirectionalValues& magn);
    virtual bool GetGyroVals(DirectionalValues& gyro);
    virtual bool GetAccelVals(DirectionalValues& accel);

protected:
    MPU6500_WE* imu;
};
#endif