#include <Arduino.h>
#include "IMU_9DOF_BNO08x.h"
#include "IMU_9DOF_9250.h"

IMU_9DOF_BNO08x bno;
IMU_9DOF_9250 mpu;

uint16_t count;
uint32_t startTime;

IMU_9DOF::Quaternion quaternion;
IMU_9DOF::Euler euler;
IMU_6DOF::DirectionalValues grav;
IMU_6DOF::DirectionalValues accel;
IMU_6DOF::DirectionalValues gyro;
IMU_6DOF::DirectionalValues magnet;

uint32_t lastUpdate = micros();

void setup()
{
    Serial.begin(9600);
    while (!Serial);

    bno.Init();
    mpu.Init();
}

void loop()
{
    startTime = millis();
    count = 0;
    while (startTime + 1000 > millis())
    {
        uint32_t measureBeginTime = micros();
        if (!bno.GetMagnetometerVals(magnet))
        {
            Serial.println("m");
            break;
        }
        if (!mpu.GetAccelVals(accel))
        {
            break;
        }
        if (!mpu.GetGyroVals(gyro))
        {
            break;
        }

        if (bno.GetQuaternion(quaternion, accel, gyro, magnet))
        {
            // IMU_9DOF::PrintEulerRotations(euler);

            // IMU_9DOF::PrintQuaternion(quaternion);
            bno.GetGravityVector(grav, accel, gyro, magnet);
            // IMU_6DOF::PrintReadingsAccel(grav);

            bno.CorrectAccel(accel);
            // IMU_6DOF::PrintReadingsAccel(accel);

            lastUpdate = micros();
            bool isNewPositionReady = false;

            bno.UpdatePosition(accel, lastUpdate - measureBeginTime, isNewPositionReady);
            
            // if (isNewPositionReady)
            bno.PrintCurrentPosition(accel);
        }

        count++;
    }
    
    // print how many readings we got in 1 second. Feed this in constructor of filter
    Serial.println(count);
}



void loop2()
{
    startTime = millis();
    count = 0;
    while (startTime + 1000 > millis())
    {
        if (!bno.GetMagnetometerVals(magnet))
        {
            break;
        }
        if (!mpu.GetAccelVals(accel))
        {
            break;
        }
        if (!mpu.GetGyroVals(gyro))
        {
            break;
        }

        bno.GetQuaternion(quaternion, accel, gyro, magnet);

        IMU_9DOF::PrintQuaternion(quaternion);
        count++;
    }
    
    // print how many readings we got in 1 second. Feed this in constructor of filter
    Serial.println(count);
}



