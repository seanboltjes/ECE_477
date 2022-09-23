#ifndef SENSORFUSION_H
#define SENSORFUSION_H
#include <math.h>
#include "../UnitConversions/UnitConversions.h"
#include "../MAG_LIS3MDL/MAG_LIS3MDL.h"
#include "../IMU_MPU6500/IMU_MPU6500.h"
#include "../IMU_BNO085/IMU_BNO085.h"


/**
 * @brief A fusion of both the IMU and Magnetometer. Used to perform calculations on the raw data.
 */
class SensorFusion : public IMU_BNO085
{    
public:
    // Object Management
    virtual void Init();

    virtual void Calibrate();

    // Quaternion Methods (Absolute Orientation)
    virtual bool GetEulerRotation(EulerRotations& eulerRotations, Quaternion& quaternion);
    virtual bool GetEulerRotation(EulerRotations& eulerRotations, DirectionalValues& accel, DirectionalValues& gyro, DirectionalValues& magnet);
    virtual void ConvertQuaternionToRotationMatrix(Quaternion& quaternion, BLA::Matrix<4, 4>& rotationMatrix);
    virtual void ConvertQuaternionToEulerAngles(Quaternion& quaternion, EulerRotations& euler);
    virtual void ConvertLocalToGlobalCoords(DirectionalValues& uncorrectedAccel, DirectionalValues& correctedAceel, BLA::Matrix<4, 4>& rotationMatrix);
    virtual void ConvertLocalToGlobalCoords(DirectionalValues& uncorrectedAccel, DirectionalValues& correctedAccel, EulerRotations& euler);

    // Position Methods
    // virtual bool GetGravityVector(DirectionalValues& gravityVector, Quaternion& quaternion);
    virtual void CorrectAccel(DirectionalValues& accel);
    virtual void UpdatePosition(DirectionalValues& correctedAccel, uint32_t timeSinceLastUpdate_us);

    // Print Methods
    static void PrintGravityVector(DirectionalValues& gravity);
    static void PrintEulerRotations(EulerRotations& eulerRotations);
    virtual void PrintCurrentPosition();
    virtual void GetAndPrintAllReadings();
    virtual void PrintDetailedDeadReckoning(DirectionalValues& accel);

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
};

#endif