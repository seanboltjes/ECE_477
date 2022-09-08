#include "IMU_MPU6500.h"



/**
 * @brief Initializes the IMU
 */
void IMU_MPU6500::InitIMU()
{
    Wire.begin();
    imu = new MPU6500_WE(0x68);

    while (!imu->init()){
        Serial.println("MPU6500 does not respond");
        delay(100);
    }
    

    Serial.println("Position your MPU6500 flat and don't move it - calibrating...");
    delay(1000);
    imu->autoOffsets();
    Serial.println("Done!");
  
    imu->setSampleRateDivider(5);
    imu->setAccRange(MPU6500_ACC_RANGE_4G);
    imu->enableAccDLPF(true);
    imu->setAccDLPF(MPU6500_DLPF_6); 

    
    imu->setGyrDLPF(MPU6500_DLPF_6);
    imu->setSampleRateDivider(99);
    imu->setGyrRange(MPU6500_GYRO_RANGE_500);

    imu->sleep(false);

    Serial.println("MPU6500 is connected");
}


/**
 * @brief Destructor. Frees allocated memory
 */
IMU_MPU6500::~IMU_MPU6500()
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
bool IMU_MPU6500::GetGyroVals(DirectionalValues& gyro)
{
    xyzFloat gyrovals = imu->getGyrValues();

    gyro.x = gyrovals.x;
    gyro.y = gyrovals.y;
    gyro.z = gyrovals.z;
    
    return true;
}


/**
 * @brief Returns a struct with the current accel values (in g's)
 * 
 * @param accel the struct to be populated with acceleration data
 * 
 * @return bool - true if successful false if not successful
 */
bool IMU_MPU6500::GetAccelVals(DirectionalValues& accel)
{
    xyzFloat floats = imu->getGValues();

    accel.x = floats.x;
    accel.y = floats.y;
    accel.z = floats.z;
    
    return true;
}


/**
 * @brief Prints the accel values to the terminal
 *
 * @param accel DirectionalValues struct that contains the readings to print
 */
void IMU_MPU6500::PrintReadingsAccel(DirectionalValues &accel)
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
void IMU_MPU6500::PrintReadingsGyro(DirectionalValues &gyro)
{
    Serial.print("Gyro: ");
    Serial.print(gyro.x);
    Serial.print(", ");
    Serial.print(gyro.y);
    Serial.print(", ");
    Serial.println(gyro.z);
}