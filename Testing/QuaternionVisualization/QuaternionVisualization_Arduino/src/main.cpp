#include <Arduino.h>
#include "IMU_9DOF_BNO08x.h"

IMU_9DOF_BNO08x imu;

uint16_t count;
uint32_t startTime;

void setup()
{
    Serial.begin(9600);
    while (!Serial);

    imu.Init();
}


void loop()
{
    startTime = millis();
    count = 0;
    while (startTime + 1000 > millis())
    {
        IMU_9DOF::Quaternion quaternion;

        imu.GetQuaternion(quaternion);

        IMU_9DOF::PrintQuaternion(quaternion);
        count++;
    }
    
    // print how many readings we got in 1 second. Feed this in constructor of filter
    Serial.println(count);
}



