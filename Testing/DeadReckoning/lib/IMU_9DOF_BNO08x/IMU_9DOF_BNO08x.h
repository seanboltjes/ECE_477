#ifndef IMU_9DOF_BNO08X_H
#define IMU_9DOF_BNO08X_H
#include <math.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include "../IMU_9DOF/IMU_9DOF.h"

/*
Class for the BNO08x 9-DOF IMU sensors 
*/

/**
 * @brief The class to control the Adafruit BNO085 IMU module and any other BNO08x modules
 */
class IMU_9DOF_BNO08x : public IMU_9DOF
{    
public:
    virtual void Init();
    virtual ~IMU_9DOF_BNO08x();

    virtual bool GetGyroVals(DirectionalValues& gyro);
    virtual bool GetAccelVals(DirectionalValues& accel);
    virtual bool GetMagnetometerVals(DirectionalValues& magn);

protected:
    Adafruit_BNO08x* imu;

private:
    uint16_t TIMEOUT_MS = 1500;

    bool SetReports();
};
#endif