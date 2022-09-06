#include "IMU_6DOF_LSM6DSR.h"


/**
 * @brief Initializes the IMU and blocks until it is ready
 */
void IMU_6DOF_LSM6DSR::Init()
{
    Wire.begin();
    imu = new LSM6DSRSensor(&Wire);

    while (imu->begin())
    {
        Serial.println("IMU INIT FAIL");
        delay(100);
    }

    Serial.println("INIT IMU SUCCESS");
    imu->Enable_X();
    imu->Enable_G();
}


/**
 * @brief Initializes the IMU and blocks until it is ready
 * 
 * @param buzzer
 */
void IMU_6DOF_LSM6DSR::Init(Buzzer &buzzer)
{
    Wire.begin();
    imu = new LSM6DSRSensor(&Wire);

    while (!imu->begin())
    {
        Serial.println("IMU INIT FAIL");
        buzzer.BuzzErrorIMU();
        delay(100);
    }

    Serial.println("INIT IMU SUCCESS");
    imu->Enable_X();
    imu->Enable_G();
}


/**
 * @brief Destroy the IMU_6DOF_LSM6DSR object
 * 
 */
IMU_6DOF_LSM6DSR::~IMU_6DOF_LSM6DSR()
{
    delete imu;
}


/**
 * @brief Returns a struct with the current gyro values (in degrees/second)
 * 
 * @param gyro the struct to be populated with gyroscope data
 * 
 * @return bool - true if successful false if not successful
 */
bool IMU_6DOF_LSM6DSR::GetGyroVals(DirectionalValues& gyro)
{
    int32_t gyroscope[3];

    LSM6DSRStatusTypeDef status = imu->Get_G_Axes(gyroscope);
    
    gyro.x = gyroscope[0] * .001;
    gyro.y = gyroscope[1] * .001;
    gyro.z = gyroscope[2] * .001;

    return status == LSM6DSR_OK;
}


/**
 * @brief Returns a struct with the current accel values (in g's)
 * 
 * @param accel the struct to be populated with acceleration data
 * 
 * @return bool - true if successful false if not successful
 */
bool IMU_6DOF_LSM6DSR::GetAccelVals(DirectionalValues& accel)
{
    int32_t accelerometer[3];

    LSM6DSRStatusTypeDef status = imu->Get_X_Axes(accelerometer);

    accel.x = accelerometer[0] * .001;
    accel.y = accelerometer[1] * .001;
    accel.z = accelerometer[2] * .001;

    return status == LSM6DSR_OK;
}