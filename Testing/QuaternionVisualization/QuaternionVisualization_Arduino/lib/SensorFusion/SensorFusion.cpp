#include "SensorFusion.h"




/**
 * @brief Initizalizes both the BNO085
 */
void SensorFusion::Init()
{
    InitIMU();
}


/**
 * @brief Blocks until the device is calibrated. This works by waiting until acceleration is completely corrected by gravity
 */
void SensorFusion::Calibrate()
{
    DirectionalValues magnet;
    DirectionalValues accel;
    DirectionalValues gyro;
    DirectionalValues gravity;
    Quaternion quaternion;

    Serial.println("Calibrating... hold still");
    delay(1000);

    uint8_t count = 0;

    while (count < 50)
    {
        // Try and get the Quaternion
        if (GetQuaternion(quaternion))
        {
            // Get the gravity vector and correct the acceleration
            GetGravityVector(gravity);
            CorrectAccel(accel);

            if (accel.x <= 0.01 && accel.x >= -0.01 && accel.y <= 0.01 && accel.y >= -0.01 && accel.z <= 0.01 && accel.z >= -0.01)
            {
                count++;
            }
        }
    }

    Serial.println("Device Calibrated!");
}


// /**
//  * @brief Gets the gravity vector. Basically tells the force of gravity in all 3 directions
//  * 
//  * @param gravityVector this is populated with the newly calculated forces of gravity
//  * @param quaternion the quaternion to use for getting the vector
//  * 
//  * @returns bool - if the gravityVector struct was successfully (true) populated or not (false)
//  */
// bool SensorFusion::GetGravityVector(DirectionalValues& gravityVector, Quaternion& quaternion)
// {
//     // float x, y, z;

//     // filter->getGravityVector(&x, &y, &z);

//     // gravityVector.x = x;
//     // gravityVector.y = y;
//     // gravityVector.z = z;


//     float i, j, k, r;

//     i = quaternion.i;
//     j = quaternion.j;
//     k = quaternion.k;
//     r = quaternion.real;


//     gravityVector.x = 2 * (i * k + r * j);
//     gravityVector.y = 2 * (j * k - r * i);
//     gravityVector.z = 1 - (2 * (i * i - j * j));









    
//     // EulerRotations eulerRotation;
//     // GetEulerRotation(eulerRotation, quaternion);

//     // gravityVector.z = cos(UnitConversions::DegreesToRadians(eulerRotation.roll)) * cos(UnitConversions::DegreesToRadians(eulerRotation.pitch));
//     // gravityVector.y = sin(UnitConversions::DegreesToRadians(eulerRotation.roll)) * cos(UnitConversions::DegreesToRadians(eulerRotation.pitch));
//     // gravityVector.x = -1 * sin(UnitConversions::DegreesToRadians(eulerRotation.pitch));

//     // // gravityVector.x = cos(UnitConversions::DegreesToRadians(eulerRotation.yaw)) * cos(UnitConversions::DegreesToRadians(eulerRotation.pitch));
//     // // gravityVector.y = sin(UnitConversions::DegreesToRadians(eulerRotation.yaw)) * cos(UnitConversions::DegreesToRadians(eulerRotation.pitch));
//     // // gravityVector.z = -1 * sin(UnitConversions::DegreesToRadians(eulerRotation.pitch));
    
//     // gravX = gravityVector.x;
//     // gravY = gravityVector.y;
//     // gravZ = gravityVector.z;

//     return true;
// }


/**
 * @brief Corrects the raw acceleration readings and repopulates the passed in struct with the corrected values
 * 
 * @param accel the acceleration readings to correct. When the function call is finished this will now be populated with the corrected values
 */
void SensorFusion::CorrectAccel(DirectionalValues& accel)
{
    accel.x -= gravX;
    accel.y -= gravY;
    accel.z -= gravZ;
}


/**
 * @brief Updates the current position estimate using Dead Reckoning
 * 
 * @param correctedAccel the acceleration after it has been corrected from gravity
 * @param timeSinceLastUpdate_us the time in microseconds since this method was last called.
 */
void SensorFusion::UpdatePosition(DirectionalValues& correctedAccel, uint32_t timeSinceLastUpdate_us)
{
    double accelTermX, accelTermY, accelTermZ;

    const double errorMargin = 0.02;
    const double neg_errorMargin = -0.02;

    const double errorMarginVel = 0.0001;
    const double neg_errorMarginVel = -0.0001;

    if (correctedAccel.x < errorMargin && correctedAccel.x > neg_errorMargin)
    {
        correctedAccel.x = 0.0f;
        numZeroX++;
    }
    if (correctedAccel.y < errorMargin && correctedAccel.y > neg_errorMargin)
    {
        correctedAccel.y = 0.0f;
        numZeroY++;
    }
    if (correctedAccel.z < errorMargin && correctedAccel.z > neg_errorMargin)
    {
        correctedAccel.z = 0.0f;
        numZeroZ++;
    }


    if (velX < errorMarginVel && velX > neg_errorMarginVel) 
    {
        velX = 0.0f;
    }
    if (velY < errorMarginVel && velY > neg_errorMarginVel) 
    {
        velY = 0.0f;
    }
    if (velZ < errorMarginVel && velZ > neg_errorMarginVel) 
    {
        velZ = 0.0f;
    }
    

    if (numZeroX >= 3)
    {
        numZeroX = 0;
        velX = 0.0f;
        // posX = 0.0f;
    }
    if (numZeroY >= 3)
    {
        numZeroY = 0;
        velY = 0.0f;
        // posY = 0.0f;
    }
    if (numZeroZ >= 3)
    {
        numZeroZ = 0;
        velZ = 0.0f;
        // posZ = 0.0f;
    }

    samplesTakenSinceReset++;


    if (GetNetAcceleration(correctedAccel) > 1 && samplesTakenSinceReset > 100)
    {
        posX = 0;
        posY = 0;
        posZ = 0;
        velX = 0.0f;
        velY = 0.0f;
        velZ = 0.0f;
        Serial.println("777.77");

        samplesTakenSinceReset = 0;
    }

    double timeSinceLastUpdate_sec = timeSinceLastUpdate_us * 0.000001;
    double timeSinceLastUpdate_sec_squared = pow(timeSinceLastUpdate_sec, 2);
    double accelTermCoeff = timeSinceLastUpdate_sec_squared * 0.5;



    velX += timeSinceLastUpdate_sec * correctedAccel.x;
    velY += timeSinceLastUpdate_sec * correctedAccel.y;
    velZ += timeSinceLastUpdate_sec * correctedAccel.z;

    accelTermX = correctedAccel.x * accelTermCoeff;
    accelTermY = correctedAccel.y * accelTermCoeff;
    accelTermZ = correctedAccel.z * accelTermCoeff;

    posX += (velX * timeSinceLastUpdate_sec + accelTermX);
    posY += (velY * timeSinceLastUpdate_sec + accelTermY);
    posZ += (velZ * timeSinceLastUpdate_sec + accelTermZ);
}


/**
 * @brief Gets the Euler Rotation instead of Quaternion. This is a different way of seeing 3D orientation
 * 
 * @param eulerRotations this is populated with the newly updated values
 * @param accel the acceleration readings to update the filter with
 * @param gyro the gyroscope readings to update the filter with
 * @param magnet the compass readings to update the filter with
 * 
 * @returns bool - if the eulerRotations struct was successfully (true) populated or not (false)
 */
bool SensorFusion::GetEulerRotation(EulerRotations& eulerRotations, DirectionalValues& accel, DirectionalValues& gyro, DirectionalValues& magnet)
{
    Quaternion quaternion;
    if (GetQuaternion(quaternion))
    {
        float t0 = 2.0 * (quaternion.real * quaternion.i + quaternion.j * quaternion.k);
        float t1 = 1.0 - 2.0 * (quaternion.i * quaternion.i + quaternion.j * quaternion.j);
        eulerRotations.roll = UnitConversions::RadiansToDegrees(atan2(t0, t1));

        float t2 = +2.0 * (quaternion.real * quaternion.j - quaternion.k * quaternion.i);
        
        t2 = t2 > 1 ? 1.0 : t2;
        t2 = t2 < -1 ? -1.0 : t2;
        
        eulerRotations.pitch = UnitConversions::RadiansToDegrees(asin(t2));

        float t3 = +2.0 * (quaternion.real * quaternion.k + quaternion.i * quaternion.j);
        float t4 = +1.0 - 2.0 * (quaternion.j * quaternion.j + quaternion.k * quaternion.k);
        eulerRotations.yaw = UnitConversions::RadiansToDegrees(atan2(t3, t4));

        return true;
    }

    return false;
}


/**
 * @brief Gets the Euler Rotation instead of Quaternion. This is a different way of seeing 3D orientation
 * 
 * @param eulerRotations this is populated with the newly updated values
 * @param quaternion the quaternion to use to convert
 * 
 * @returns bool - if the eulerRotations struct was successfully (true) populated or not (false)
 */
bool SensorFusion::GetEulerRotation(EulerRotations& eulerRotations, Quaternion& quaternion)
{
    float t0 = 2.0 * (quaternion.real * quaternion.i + quaternion.j * quaternion.k);
    float t1 = 1.0 - 2.0 * (quaternion.i * quaternion.i + quaternion.j * quaternion.j);
    eulerRotations.roll = UnitConversions::RadiansToDegrees(atan2(t0, t1));

    float t2 = 2.0 * (quaternion.real * quaternion.j - quaternion.k * quaternion.i);
    
    t2 = t2 > 1 ? 1.0 : t2;
    t2 = t2 < -1 ? -1.0 : t2;
    
    eulerRotations.pitch = UnitConversions::RadiansToDegrees(asin(t2));

    float t3 = 2.0 * (quaternion.real * quaternion.k + quaternion.i * quaternion.j);
    float t4 = 1.0 - 2.0 * (quaternion.j * quaternion.j + quaternion.k * quaternion.k);
    eulerRotations.yaw = UnitConversions::RadiansToDegrees(atan2(t3, t4));

    return true;
}


void SensorFusion::GetAndPrintAllReadings()
{
    DirectionalValues accel;
    DirectionalValues gyro;
    DirectionalValues magnetometer;
    
    GetAccelVals(accel);
    GetGyroVals(gyro);
    GetMagnetometerVals(magnetometer);
    
    Serial.print(accel.x);
    Serial.print("/");
    Serial.print(accel.y);
    Serial.print("/");
    Serial.print(accel.z);
    Serial.print("/");
    Serial.print(gyro.x);
    Serial.print("/");
    Serial.print(gyro.y);
    Serial.print("/");
    Serial.print(gyro.z);
    Serial.print("/");
    Serial.print(magnetometer.x);
    Serial.print("/");
    Serial.print(magnetometer.y);
    Serial.print("/");
    Serial.println(magnetometer.z);
}


void SensorFusion::PrintEulerRotations(EulerRotations& eulerRotations)
{
    Serial.print(eulerRotations.roll);
    Serial.print("/");
    Serial.print(eulerRotations.pitch);
    Serial.print("/");
    Serial.println(eulerRotations.yaw);
}


void SensorFusion::PrintCurrentPosition()
{
    Serial.print(posX);
    Serial.print("/");
    Serial.print(posY);
    Serial.print("/");
    Serial.println(posZ);
}


void SensorFusion::PrintDetailedDeadReckoning(DirectionalValues& accel)
{
    Serial.print("Position(m) (x, y, z): (");
    Serial.print(posX);
    Serial.print(", ");
    Serial.print(posY);
    Serial.print(", ");
    Serial.print(posZ);
    Serial.print(")");

    Serial.print(" Velocity(m/s): (");
    Serial.print(velX);
    Serial.print(", ");
    Serial.print(velY);
    Serial.print(", ");
    Serial.print(velZ);

    Serial.print(")  Accel(m/s/s) (x, y, z): ");
    Serial.print(accel.x);
    Serial.print(", ");
    Serial.print(accel.y);
    Serial.print(", ");
    Serial.println(accel.z);
}



void SensorFusion::PrintGravityVector(DirectionalValues& gravity)
{
    Serial.print("Gravity Vector in m/s/s (x, y, z):  ");
    Serial.print(gravity.x);
    Serial.print(", ");
    Serial.print(gravity.y);
    Serial.print(", ");
    Serial.println(gravity.z);
}