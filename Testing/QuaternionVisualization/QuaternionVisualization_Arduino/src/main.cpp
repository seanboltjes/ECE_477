#include <Arduino.h>
#include "SensorFusion.h"

// comment this out to do dead reckoning, leave it uncommented to calculate quaternions
// #define RUN_QUATERNION_CALCULATION

void DeadReckoning();
void QuaternionCalculation();

uint16_t count;
uint32_t startTime;
uint32_t startTimeCache;
uint32_t lastUpdate = micros();


// Global structs used to hold our data so that we aren't reallocating it every loop
DataStructures::Quaternion quaternion;
DataStructures::EulerRotations euler;
DataStructures::DirectionalValues grav;
DataStructures::DirectionalValues accel;
DataStructures::DirectionalValues globalAccel;
DataStructures::DirectionalValues gyro;
DataStructures::DirectionalValues magnet;

BLA::Matrix<4, 4> rotationMatrix;

SensorFusion sensors;


/**
 * @brief The setup() method is an Arduino special function. It runs automatically and is ALWAYS the first method to run
 */
void setup()
{
    // begin the Serial bus @9600 baud rate 
    Serial.begin(115200);
    // wait for the Serial bus to connect (need to open serial port on terminal or processing for this to pass)
    while (!Serial);


    // Initialize the IMU and Magnetometer
    sensors.Init();

    #ifndef RUN_QUATERNION_CALCULATION
    // Calibrate the sensors. This may take some time
    // sensors.Calibrate();
    #endif

    // Log the time when we started this measurement
    startTime = micros();
}


/**
 * @brief The loop() method is an Arduino special function. It runs automatically after setup() completes and continuously loops
 * Choose either QuaternionCalculation OR DeadReckoning
 */
void loop()
{
#ifdef RUN_QUATERNION_CALCULATION
    QuaternionCalculation();
#else
    DeadReckoning();
#endif
}


/**
 * @brief The method for testing our kind of 'dead reckoning' approach
 */
void DeadReckoning()
{
    // startTime = millis();
    // count = 0;

    // while (startTime + 1000 > millis())
    // {
    //     if (sensors.GetGravityVector(grav))
    //     {
    //         // sensors.GetGravityVector(grav);
    //         // sensors.GetGravityVector(grav);
    //         // sensors.PrintGravityVector(grav);
    //     }
    //     else
    //     {
    //         Serial.println("Err");
    //     }

    //     if (sensors.GetLinearAccelVals(grav))
    //     {
    //         // sensors.GetGravityVector(grav);
    //         // sensors.GetGravityVector(grav);
    //         // sensors.PrintReadingsAccel(grav);
    //     }
    //     else
    //     {
    //         Serial.println("Err");
    //     }

    //     if (sensors.GetQuaternion(quaternion))
    //     {
    //         // sensors.GetGravityVector(grav);
    //         // sensors.GetGravityVector(grav);
    //         // sensors.PrintQuaternion(quaternion);
    //     }
    //     else
    //     {
    //         Serial.println("Err");
    //     }
    //     count++;
    // }

    // Serial.println(count);

    // return;



    
    // Try and get acceleration, gravity, and quaternion
    // while (!sensors.GetLinearAccelVals(accel));
    while (!sensors.GetGravityVector(grav));
    // while (!sensors.GetQuaternion(quaternion));


    sensors.PrintReadingsAccel(grav);


    // sensors.ConvertQuaternionToRotationMatrix(quaternion, rotationMatrix);
    // sensors.ConvertLocalToGlobalCoords(accel, globalAccel, rotationMatrix);

    // sensors.ConvertQuaternionToEulerAngles(quaternion, euler);
    // sensors.ConvertLocalToGlobalCoords(accel, globalAccel, euler);

    // Log the time when we finished the measurement 
    // lastUpdate = micros();
    // startTimeCache = lastUpdate;

    // sensors.PrintReadingsAccel(globalAccel);

    // Serial.println(globalAccel.x);
    
    // sensors.UpdatePosition(globalAccel, lastUpdate - startTime);

    // startTime = startTimeCache;
    
    // // Print out our current position estimate
    // sensors.PrintCurrentPosition();
}


/**
 * @brief The method for testing Quaternion fetching
 */
void QuaternionCalculation()
{
    startTime = millis();
    count = 0;
    while (startTime + 1000 > millis())
    {
        sensors.GetQuaternion(quaternion);

        sensors.PrintQuaternion(quaternion);
        delay(2);
        count++;
    }
    
    // print how many readings we got in 1 second. Feed this in constructor of filter
    Serial.println(count);
}
