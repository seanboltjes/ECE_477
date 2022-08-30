#include "IMU_6DOF.h"



/**
 * @brief Get the net acceleration (magnitude across all axes)
 * 
 * @return float - net acceleration across all axes
 */
float IMU_6DOF::GetNetAccel()
{
    DirectionalValues accel;
    
    GetAccelVals(accel);
    
    return sqrt(pow(accel.x, 2) + pow(accel.y, 2) + pow(accel.z, 2));
}


/**
 * @brief Prints the accel values to the terminal
 *
 * @param accel DirectionalValues struct that contains the readings to print
 */
void IMU_6DOF::PrintReadingsAccel(DirectionalValues &accel)
{
    Serial.print("Accel: ");
    Serial.print(accel.x);
    Serial.print(", ");
    Serial.print(accel.y);
    Serial.print(", ");
    Serial.println(accel.z);
}


/**
 * @brief Prints the gyro values to the terminal
 *
 * @param gyro DirectionalValues struct that contains the readings to print
 */
void IMU_6DOF::PrintReadingsGyro(DirectionalValues &gyro)
{
    Serial.print("Gyro: ");
    Serial.print(gyro.x);
    Serial.print(", ");
    Serial.print(gyro.y);
    Serial.print(", ");
    Serial.println(gyro.z);
}