#ifndef IMU_6DOF_H
#define IMU_6DOF_H
#include <math.h>
#include <Arduino.h>
#include "../UnitConversions/UnitConversions.h"


/**
 * @brief The abstract class to use when creating new 6 Degrees of Freedom OR 9 Degrees of Freedom IMU classes
 * Have all derived IMU classes inherit this class. All these methods, structs, and variables will be available in it.
 * This standardizes all IMU modules we use.
 * 
 * Additionally, have all functions that need a IMU module take this class as a param. This way any IMU class that inherits this one will be able to be passed in
 */
class IMU_6DOF
{    
public:

    /**
     * @brief A struct containing directional readings from the IMU.
     * Gryo values are degrees/sec
     * accel values are in g's
     */
    struct DirectionalValues {
        float x;
        float y;
        float z;
    };


    virtual float GetNetAccel();

    static void PrintReadingsAccel(DirectionalValues &accel);
    static void PrintReadingsGyro(DirectionalValues &gyro);


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                    Abstract Methods (you need to implement these in child classes)
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    /**
     * @brief Initializes the IMU and blocks until it is ready
     */
    virtual void Init() = 0;

    /**
     * @brief Returns a struct with the current accel values (in g's)
     * 
     * @param accel the struct to be populated with acceleration data
     * 
     * @return bool - true if successful false if not successful
     */
    virtual bool GetAccelVals(DirectionalValues& accel) = 0;

    /**
     * @brief Returns a struct with the current gyro values (in degrees/second)
     * 
     * @param gyro the struct to be populated with gyroscope data
     * 
     * @return bool - true if successful false if not successful
     */
    virtual bool GetGyroVals(DirectionalValues& gyro) = 0;

};
#endif