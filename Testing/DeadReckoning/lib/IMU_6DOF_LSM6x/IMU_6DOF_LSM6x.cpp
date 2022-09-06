#include "IMU_6DOF_LSM6x.h"


/**
 * @brief Initializes the IMU and blocks until it is ready
 */
void IMU_6DOF_LSM6x::Init()
{
    Wire.begin();
    imu = new LSM6();

    while (!imu->init())
    {
        Serial.println("IMU INIT FAIL");
        delay(100);
    }

    Serial.println("INIT IMU SUCCESS");
    imu->enableDefault();

    // enable the accelerometer full-scale to be +-8g @ 1.66 khz sample rate - found in the datasheet, ask Sean if you have questions
    imu->writeReg(imu->CTRL1_XL, 0x8c);
}


/**
 * @brief Returns a struct with the current accel values (in g's)
 * 
 * @param accel the struct to be populated with acceleration data
 * 
 * @return bool - true if successful false if not successful
 */
bool IMU_6DOF_LSM6x::GetAccelVals(DirectionalValues& accel)
{
    imu->readAcc();

    // Ask Sean if you can't find these values in the datasheet
    accel.x = imu->a.x * 0.244 / 1000;
    accel.y = imu->a.y * 0.244 / 1000;
    accel.z = imu->a.z * 0.244 / 1000;
    
    // TODO: error checking
    return true;
}


/**
 * @brief Returns a struct with the current gyro values (in degrees/second)
 * 
 * @param gyro the struct to be populated with gyroscope data
 * 
 * @return bool - true if successful false if not successful
 */
bool IMU_6DOF_LSM6x::GetGyroVals(DirectionalValues& gyro)
{
    imu->readGyro();

    // Ask Sean if you can't find these values in the datasheet
    gyro.x = imu->g.x * 8.75 / 1000;
    gyro.y = imu->g.y * 8.75 / 1000;
    gyro.z = imu->g.z * 8.75 / 1000;

    // TODO: need error checking
    return true;
}


/**
 * @brief Destroy the IMU_6DOF_LSM6x object
 * 
 */
IMU_6DOF_LSM6x::~IMU_6DOF_LSM6x()
{
    delete imu;
}