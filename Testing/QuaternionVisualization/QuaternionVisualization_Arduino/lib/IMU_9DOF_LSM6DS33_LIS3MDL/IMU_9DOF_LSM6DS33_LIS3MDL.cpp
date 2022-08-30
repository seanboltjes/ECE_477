#include "IMU_9DOF_LSM6DS33_LIS3MDL.h"


/**
 * @brief Initializes the IMU and blocks until it is ready
 */
void IMU_9DOF_LSM6DS33_LIS3MDL::Init()
{
    Wire.begin();
    imu = new LSM6();
    lis3mdl = new Adafruit_LIS3MDL();

    while (!imu->init())
    {
        Serial.println("IMU Init Fail");
        delay(300);
    }

    while (!lis3mdl->begin_I2C())
    {
        Serial.println("Compass Init Fail");
        delay(300);
    }

    Serial.println("Init IMU Success");
    imu->enableDefault();

    // enable the accelerometer full-scale to be +-8g @ 1.66 khz sample rate - found in the datasheet, ask Sean if you have questions
    imu->writeReg(imu->CTRL1_XL, 0x8c);

    // set magnetometer data rate to 560 Hz and 8 gauss range
    lis3mdl->setDataRate(LIS3MDL_DATARATE_560_HZ);
    lis3mdl->setRange(LIS3MDL_RANGE_8_GAUSS);
    lis3mdl->setPerformanceMode(LIS3MDL_MEDIUMMODE);
    lis3mdl->setOperationMode(LIS3MDL_CONTINUOUSMODE);
}


/**
 * @brief Destroys the IMU object and the Compass object
 */
IMU_9DOF_LSM6DS33_LIS3MDL::~IMU_9DOF_LSM6DS33_LIS3MDL()
{
    delete lis3mdl;
    delete imu;
}


/**
 * @brief Returns a struct with the current magnetometer values (in micro Tesla)
 * 
 * @param magn the struct to be populated with magnetometer data
 * 
 * @return bool - true if successful false if not successful
 */
bool IMU_9DOF_LSM6DS33_LIS3MDL::GetMagnetometerVals(DirectionalValues& magn)
{
    sensors_event_t mag;
    bool isSuccess = lis3mdl->getEvent(&mag);

    if (isSuccess)
    {
        magn.x = mag.magnetic.x;
        magn.y = mag.magnetic.y;
        magn.z = mag.magnetic.z;
    }

    return isSuccess;
}


/**
 * @brief Returns a struct with the current accel values (in g's)
 * 
 * @param accel the struct to be populated with acceleration data
 * 
 * @return bool - true if successful false if not successful
 */
bool IMU_9DOF_LSM6DS33_LIS3MDL::GetAccelVals(DirectionalValues& accel)
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
bool IMU_9DOF_LSM6DS33_LIS3MDL::GetGyroVals(DirectionalValues& gyro)
{
    imu->readGyro();

    // Ask Sean if you can't find these values in the datasheet
    gyro.x = imu->g.x * 8.75 / 1000;
    gyro.y = imu->g.y * 8.75 / 1000;
    gyro.z = imu->g.z * 8.75 / 1000;

    // TODO: need error checking
    return true;
}
