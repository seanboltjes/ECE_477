#include "IMU_3DOF_MAGNETOMETER.h"



void IMU_3DOF_MAGNETOMETER::Init()
{
    Wire.begin();
    Serial.println("here");
    lsm = new Adafruit_LSM303();
    Serial.println("here1");

    while (!lsm->begin())
    {
        Serial.println("Oops ... unable to initialize the LSM303. Check your wiring!");
        delay(500);
    }
}


IMU_3DOF_MAGNETOMETER::~IMU_3DOF_MAGNETOMETER()
{
    delete lsm;
}

bool IMU_3DOF_MAGNETOMETER::GetMagnetometerVals(IMU_6DOF::DirectionalValues& gyro)
{
    lsm->read();

    gyro.x = lsm->magData.x;
    gyro.y = lsm->magData.y;
    gyro.z = lsm->magData.z;
}