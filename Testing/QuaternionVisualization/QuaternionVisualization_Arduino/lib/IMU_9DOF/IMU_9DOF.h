#ifndef IMU_9DOF_H
#define IMU_9DOF_H
#include <Adafruit_AHRS.h>
#include "../IMU_6DOF/IMU_6DOF.h"


/**
 * @brief The abstract class to use when creating new 9 Degrees of Freedom OR IMU classes
 * Have all derived 9 DoF IMU classes inherit this class. All these methods, structs, and variables will be available in it.
 * This standardizes all IMU modules we use.
 * 
 * Additionally, have all functions that need a 9 DoF IMU module take this class as a param. This way any 9 DoF IMU class that inherits this one will be able to be passed in
 */
class IMU_9DOF : public IMU_6DOF
{    
public:
    struct Quaternion
    {
        float real; // sometimes denoted W
        float i;    // sometimes denoted X
        float j;    // sometimes denoted Y
        float k;    // sometimes denoted Z
    };

    struct Euler
    {
        float roll;
        float pitch;
        float yaw;
    };

    IMU_9DOF();
    virtual bool GetEulerRotation(Euler& eulerRotations, DirectionalValues accel, DirectionalValues gyro, DirectionalValues magnet);
    virtual bool GetQuaternion(Quaternion& quaternion);
    virtual bool GetGravityVector(DirectionalValues& gravityVector, DirectionalValues accel, DirectionalValues gyro, DirectionalValues magnet);
    virtual bool GetQuaternion(Quaternion& quaternion, DirectionalValues accel, DirectionalValues gyro, DirectionalValues magnet);
    static void PrintReadingsMagnetometer(DirectionalValues& magn);
    static void PrintQuaternion(Quaternion& quaternion);
    static void PrintEulerRotations(Euler& eulerRotations);
    void PrintCurrentPosition(DirectionalValues& accel);
    void GetAndPrintAllReadings();

    void CorrectAccel(DirectionalValues& accel);
    void UpdatePosition(DirectionalValues& correctedAccel, uint32_t timeSinceLastUpdate_us);



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                    Abstract Methods (you need to implement these in child classes)
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    /**
     * @brief Returns a struct with the current magnetometer values (in micro Tesla)
     * 
     * @param magn the struct to be populated with magnetometer data
     * 
     * @return bool - true if successful false if not successful
     */
    virtual bool GetMagnetometerVals(DirectionalValues& magn) = 0;

protected:
    Adafruit_Mahony* filter;

private:
    uint32_t microsSinceLastTimeValuesUpdated;
    float deltat = 0.0f;
    
    float r0 = 0;
    float r1 = 0;
    float i0 = 0;
    float i1 = 0;
    float j0 = 0;
    float j1 = 0;
    float k0 = 0;
    float k1 = 0;

    float gravX;
    float gravY;
    float gravZ;

    float posX = 0;
    float posY = 0;
    float posZ = 0;
    float velX = 0;
    float velY = 0;
    float velZ = 0;



    virtual void ComputeQuaternion(Quaternion& quaternion, DirectionalValues& accel, DirectionalValues& gyro, DirectionalValues& magnet);
};
#endif