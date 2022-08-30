#include "IMU_9DOF_BNO08x.h"



/**
 * @brief Initializes the IMU and blocks until it is ready
 */
void IMU_9DOF_BNO08x::Init()
{
    Wire.begin();
    imu = new Adafruit_BNO08x();

    while (!imu->begin_I2C())
    {
        Serial.println("IMU Init Fail");
        delay(100);
    }

    while (!SetReports())
    {
        Serial.println("IMU Setup Fail");
        delay(100);
    }

    filter = new Adafruit_NXPSensorFusion();
    filter->begin(29);    // 48  50  29

    Serial.println("IMU Init Success");
}


/**
 * @brief Destroys the IMU object
 */
IMU_9DOF_BNO08x::~IMU_9DOF_BNO08x()
{
    delete imu;
}


/**
 * @brief Sets the reports (data points) that the BNO will produce.
 * 
 * @return bool - true if successful false if not successful
 */
bool IMU_9DOF_BNO08x::SetReports() 
{
    bool allSucces = true;

    if (!imu->enableReport(SH2_ACCELEROMETER)) {
        Serial.println("Could not enable accelerometer");
        allSucces = false;
    }
    if (!imu->enableReport(SH2_GYROSCOPE_CALIBRATED)) {
        Serial.println("Could not enable gyroscope");
        allSucces = false;
    }
    if (!imu->enableReport(SH2_MAGNETIC_FIELD_CALIBRATED)) {
        Serial.println("Could not enable magnetic field calibrated");
        allSucces = false;
    }

    return allSucces;
}



bool IMU_9DOF_BNO08x::GetQuaternion(Quaternion& quaternion)
{
    DirectionalValues accel;
    DirectionalValues gyro;
    DirectionalValues magnetometer;

    if (!(GetAccelVals(accel) && GetGyroVals(gyro) && GetMagnetometerVals(magnetometer)))
    {
        // one of the values was not successfully grabbed, can't calculate quaternion
        Serial.println("Can't get sensor vals");
        return false;
    }

    filter->update(gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z, magnetometer.x, magnetometer.y, magnetometer.z);

    float x;
    float y;
    float z;
    float w;

    filter->getQuaternion(&w, &x, &y, &z);

    quaternion.real = w;
    quaternion.i = x;
    quaternion.j = y;
    quaternion.k = z;

    return true;
}

/**
 * @brief Returns a struct with the current accel values (in g's)
 * 
 * @param accel the struct to be populated with acceleration data
 * 
 * @return bool - true if successful false if not successful
 */
bool IMU_9DOF_BNO08x::GetAccelVals(DirectionalValues& accel)
{
    sh2_SensorValue_t sensorValue;
    uint32_t startTime = millis();

    while (millis() < startTime + TIMEOUT_MS)
    {
        if (imu->getSensorEvent(&sensorValue)) 
        {
            if (sensorValue.sensorId == SH2_ACCELEROMETER)
            {
                accel.x = sensorValue.un.accelerometer.x;
                accel.y = sensorValue.un.accelerometer.y;
                accel.z = sensorValue.un.accelerometer.z;

                return true;
            }
        }
    }

    return false;
}


/**
 * @brief Returns a struct with the current magnetometer values (in micro Tesla)
 * 
 * @param magn the struct to be populated with magnetometer data
 * 
 * @return bool - true if successful false if not successful
 */
bool IMU_9DOF_BNO08x::GetMagnetometerVals(DirectionalValues& magn)
{
    sh2_SensorValue_t sensorValue;
    uint32_t startTime = millis();

    while (millis() < startTime + TIMEOUT_MS)
    {
        if (imu->getSensorEvent(&sensorValue)) 
        {
            if (sensorValue.sensorId == SH2_MAGNETIC_FIELD_CALIBRATED)
            {
                magn.x = sensorValue.un.magneticField.x;
                magn.y = sensorValue.un.magneticField.y;
                magn.z = sensorValue.un.magneticField.z;

                return true;
            }
        }
    }

    return false;
}


/**
 * @brief Returns a struct with the current gyro values (in degrees/second)
 * 
 * @param gyro the struct to be populated with gyroscope data
 * 
 * @return bool - true if successful false if not successful
 */
bool IMU_9DOF_BNO08x::GetGyroVals(DirectionalValues& gyro)
{
    sh2_SensorValue_t sensorValue;
    uint32_t startTime = millis();

    while (millis() < startTime + TIMEOUT_MS)
    {
        if (imu->getSensorEvent(&sensorValue)) 
        {
            if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED)
            {
                gyro.x = UnitConversions::RadiansToDegrees(sensorValue.un.gyroscope.x);
                gyro.y = UnitConversions::RadiansToDegrees(sensorValue.un.gyroscope.y);
                gyro.z = UnitConversions::RadiansToDegrees(sensorValue.un.gyroscope.z);

                return true;
            }
        }
    }

    return false;
}




