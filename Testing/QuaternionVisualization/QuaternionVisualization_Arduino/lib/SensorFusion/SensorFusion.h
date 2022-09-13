#ifndef SENSORFUSION_H
#define SENSORFUSION_H
#include <math.h>
#include <Adafruit_AHRS.h>
#include <Adafruit_BNO08x.h>
#include "../UnitConversions/UnitConversions.h"
#include "../MAG_LIS3MDL/MAG_LIS3MDL.h"
#include "../IMU_MPU6500/IMU_MPU6500.h"


/**
 * @brief A fusion of both the IMU and Magnetometer. Used to perform calculations on the raw data.
 */
class SensorFusion : public IMU_MPU6500, public MAG_LIS3MDL
{    
public:
    // Object Management
    SensorFusion();
    SensorFusion(float samplingRate);
    virtual ~SensorFusion();
    virtual void Init();

    virtual void Calibrate();

    // Quaternion Methods (Absolute Orientation)
    virtual bool GetEulerRotation(EulerRotations& eulerRotations, Quaternion& quaternion);
    virtual bool GetEulerRotation(EulerRotations& eulerRotations, DirectionalValues& accel, DirectionalValues& gyro, DirectionalValues& magnet);
    virtual bool GetQuaternion(Quaternion& quaternion);
    virtual bool GetQuaternion(Quaternion& quaternion, DirectionalValues& accel, DirectionalValues& gyro, DirectionalValues& magnet);

    // Position Methods
    virtual bool GetGravityVector(DirectionalValues& gravityVector, Quaternion& quaternion);
    virtual void CorrectAccel(DirectionalValues& accel);
    virtual void UpdatePosition(DirectionalValues& correctedAccel, uint32_t timeSinceLastUpdate_us);

    // Print Methods
    static void PrintGravityVector(DirectionalValues& gravity);
    static void PrintQuaternion(Quaternion& quaternion);
    static void PrintEulerRotations(EulerRotations& eulerRotations);
    virtual void PrintCurrentPosition();
    virtual void GetAndPrintAllReadings();
    virtual void PrintDetailedDeadReckoning(DirectionalValues& accel);


protected:
    Adafruit_Mahony* filter;

private:
    uint32_t microsSinceLastTimeValuesUpdated;
    float deltat = 0.0f;

    float gravX;
    float gravY;
    float gravZ;

    float posX = 0;
    float posY = 0;
    float posZ = 0;
    float velX = 0;
    float velY = 0;
    float velZ = 0;

    uint16_t numZeroX = 0;
    uint16_t numZeroY = 0;
    uint16_t numZeroZ = 0;

    float posXCache = 0;
    float posYCache = 0;
    float posZCache = 0;

    uint16_t samplesTakenSinceReset = 0;

    Adafruit_BNO08x* imu;

    bool SetReports();
    virtual void ComputeQuaternion(Quaternion& quaternion, DirectionalValues& accel, DirectionalValues& gyro, DirectionalValues& magnet);
};

#endif