#ifndef IMU_BNO085_H
#define IMU_BNO085_H
#include <math.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include "../DataStructures/DataStructures.h"


/**
 * @brief class for the BNO085 9 DoF IMU
 */
class IMU_BNO085 : public DataStructures
{    
public:
    // Getters
    virtual bool GetGyroVals(DirectionalValues& gyro);
    virtual bool GetAccelVals(DirectionalValues& accel);
    virtual bool GetLinearAccelVals(DirectionalValues& accel);
    virtual bool GetMagnetometerVals(DirectionalValues& magn);
    virtual bool GetQuaternion(Quaternion& quaternion);
    virtual bool GetGravityVector(DirectionalValues& gravity);
    virtual float GetNetAcceleration(DirectionalValues& accel);

    // Print Methods
    virtual void PrintReadingsMagnetometer(DirectionalValues& magn);
    virtual void PrintReadingsGyro(DirectionalValues& gyro);
    virtual void PrintReadingsAccel(DirectionalValues& accel);
    virtual void PrintQuaternion(Quaternion& quaternion);
protected:
    virtual void InitIMU();
    virtual ~IMU_BNO085();

    Adafruit_BNO08x* imu;

private:
    virtual bool SetReports();
};
#endif