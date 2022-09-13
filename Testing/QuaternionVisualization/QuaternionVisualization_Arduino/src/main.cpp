#include <Arduino.h>
#include "SensorFusion.h"

// comment this out to do dead reckoning, leave it uncommented to calculate quaternions
// #define RUN_QUATERNION_CALCULATION

void DeadReckoning();
void QuaternionCalculation();

uint16_t count;
uint32_t startTime;
uint32_t lastUpdate = micros();


// Global structs used to hold our data so that we aren't reallocating it every loop
DataStructures::Quaternion quaternion;
DataStructures::EulerRotations euler;
DataStructures::DirectionalValues grav;
DataStructures::DirectionalValues accel;
DataStructures::DirectionalValues gyro;
DataStructures::DirectionalValues magnet;


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

    // hello world
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
    // Log the time when we started this measurement
    uint32_t measureBeginTime = micros();
    
    // Try and get acceleration
    while (!sensors.GetAccelVals(accel));

    // Log the time when we finished the measurement 
    lastUpdate = micros();
    sensors.UpdatePosition(accel, lastUpdate - measureBeginTime);
    
    // Print out our current position estimate
    sensors.PrintCurrentPosition();
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
