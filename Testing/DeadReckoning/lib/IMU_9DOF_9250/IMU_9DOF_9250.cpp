#include "IMU_9DOF_9250.h"

void IMU_9DOF_9250::Init()
{
    Serial.println("MPU9250 is attempt");
    delay(100);
    

    Wire.begin();
    Serial.println("I2C begin");
    imu = new MPU6500_WE(0x68);

    while (!imu->init()){
        Serial.println("MPU9250 does not respond");
        delay(100);
    }
    

    Serial.println("Position you MPU9250 flat and don't move it - calibrating...");
    delay(1000);
    imu->autoOffsets();
    Serial.println("Done!");
  
    imu->setSampleRateDivider(5);
    imu->setAccRange(MPU9250_ACC_RANGE_2G);
    imu->enableAccDLPF(true);
    imu->setAccDLPF(MPU9250_DLPF_6);

    
    imu->setGyrDLPF(MPU9250_DLPF_6);
    imu->setSampleRateDivider(99);
    imu->setGyrRange(MPU9250_GYRO_RANGE_250);

    imu->sleep(false);

    Serial.println("MPU9250 is connected");
}


IMU_9DOF_9250::~IMU_9DOF_9250()
{
    delete imu;
}


bool IMU_9DOF_9250::GetMagnetometerVals(DirectionalValues& magn)
{
    return true;
}


bool IMU_9DOF_9250::GetGyroVals(DirectionalValues& gyro)
{
    // xyzFloat floats = imu->getAngles();
    xyzFloat gyrovals = imu->getGyrValues();

    // float pitch = imu->getPitch();
    // float roll = imu->getRoll();

    // Serial.print("Pitch Roll:");
    // Serial.print(pitch);
    // Serial.print(", ");
    // Serial.println(roll);

    gyro.x = gyrovals.x;
    gyro.y = gyrovals.y;
    gyro.z = gyrovals.z;
    
    return true;
}


bool IMU_9DOF_9250::GetAccelVals(DirectionalValues& accel)
{
    xyzFloat floats = imu->getGValues();

    accel.x = floats.x;
    accel.y = floats.y;
    accel.z = floats.z;
    
    return true;
}

