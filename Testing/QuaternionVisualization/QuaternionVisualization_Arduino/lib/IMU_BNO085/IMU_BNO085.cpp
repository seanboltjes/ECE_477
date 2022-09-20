#include "IMU_BNO085.h"



/**
 * @brief Initializes the IMU
 */
void IMU_BNO085::InitIMU()
{
    Wire.setClock(400000);
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


    Serial.println("IMU Init Success");
}


/**
 * @brief Destructor. Frees allocated memory
 */
IMU_BNO085::~IMU_BNO085()
{
    delete imu;
}


/**
 * @brief Sets the reports (data points) that the BNO will produce.
 * 
 * @return bool - true if successful false if not successful
 */
bool IMU_BNO085::SetReports() 
{
    bool allSucces = true;

    // if (!imu->enableReport(SH2_ACCELEROMETER, 1000)) {
    //     Serial.println("Could not enable accelerometer");
    //     allSucces = false;
    // }
    if (!imu->enableReport(SH2_LINEAR_ACCELERATION, 1000)) {
        Serial.println("Could not enable linear accelerometer");
        allSucces = false;
    }
    if (!imu->enableReport(SH2_GRAVITY, 1000)) {      //-- Can't get this one to update at a fast rate with LINEAR_ACCELERATION :(
        Serial.println("Could not enable gravity");
        allSucces = false;
    }
    // if (!imu->enableReport(SH2_GYROSCOPE_CALIBRATED)) {
    //     Serial.println("Could not enable gyroscope");
    //     allSucces = false;
    // }
    // if (!imu->enableReport(SH2_MAGNETIC_FIELD_CALIBRATED)) {
    //     Serial.println("Could not enable magnetic field calibrated");
    //     allSucces = false;
    // }
    if (!imu->enableReport(SH2_ROTATION_VECTOR, 20000)) {
        Serial.println("Could not enable rotation vector");
        allSucces = false;
    }

    return allSucces;
}


/**
 * @brief Returns a struct with the current gyro values (in degrees/second)
 * 
 * @param gyro the struct to be populated with gyroscope data
 * 
 * @return bool - true if successful false if not successful
 */
bool IMU_BNO085::GetGyroVals(DirectionalValues& gyro)
{
    sh2_SensorValue_t sensorValue;
    uint32_t startTime = millis();

    while (millis() < startTime + 1500)
    {
        if (imu->getSensorEvent(&sensorValue)) 
        {
            if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED)
            {
                gyro.x = sensorValue.un.gyroscope.x;
                gyro.y = sensorValue.un.gyroscope.y;
                gyro.z = sensorValue.un.gyroscope.z;

                return true;
            }
        }
    }

    return false;
}


/**
 * @brief Returns a struct with the current accel values (in g's)
 * 
 * @param accel the struct to be populated with acceleration data
 * 
 * @return bool - true if successful false if not successful
 */
bool IMU_BNO085::GetLinearAccelVals(DirectionalValues& accel)
{
    sh2_SensorValue_t sensorValue;
    uint32_t startTime = millis();

    while (millis() < startTime + 1500)
    {
        if (imu->getSensorEvent(&sensorValue)) 
        {
            if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION)
            {
                accel.x = sensorValue.un.linearAcceleration.x;
                accel.y = sensorValue.un.linearAcceleration.y;
                accel.z = sensorValue.un.linearAcceleration.z;
                
                return true;
            }
        }
    }

    return false;
}


/**
 * @brief Returns a struct with the current accel values (in g's)
 * 
 * @param accel the struct to be populated with acceleration data
 * 
 * @return bool - true if successful false if not successful
 */
bool IMU_BNO085::GetAccelVals(DirectionalValues& accel)
{
    sh2_SensorValue_t sensorValue;
    uint32_t startTime = millis();

    while (millis() < startTime + 1500)
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
bool IMU_BNO085::GetMagnetometerVals(DirectionalValues& magn)
{
    sh2_SensorValue_t sensorValue;
    uint32_t startTime = millis();

    while (millis() < startTime + 1500)
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
 * @brief Gets a Quaternion
 * 
 * @param gravityVector this is populated with the new Quaternion
 * 
 * @returns bool - if the Quaternion struct was successfully (true) populated or not (false)
 */
bool IMU_BNO085::GetQuaternion(Quaternion& quaternion)
{
    sh2_SensorValue_t sensorValue;
    uint32_t startTime = millis();

    while (millis() < startTime + 1500)
    {
        if (imu->getSensorEvent(&sensorValue)) 
        {
            if (sensorValue.sensorId == SH2_ROTATION_VECTOR)
            {
                quaternion.real = sensorValue.un.rotationVector.real;
                quaternion.i = sensorValue.un.rotationVector.i;
                quaternion.j = sensorValue.un.rotationVector.j;
                quaternion.k = sensorValue.un.rotationVector.k;

                return true;
            }
        }
    }

    return false;
}


bool IMU_BNO085::GetGravityVector(DirectionalValues& gravity)
{
    sh2_SensorValue_t sensorValue;
    uint32_t startTime = millis();

    while (millis() < startTime + 1500)
    {
        if (imu->getSensorEvent(&sensorValue)) 
        {
            if (sensorValue.sensorId == SH2_GRAVITY)
            {
                gravity.x = sensorValue.un.gravity.x;
                gravity.y = sensorValue.un.gravity.y;
                gravity.z = sensorValue.un.gravity.z;

                return true;
            }
        }
    }

    return false;

    // DirectionalValues accel;
    // DirectionalValues linAccel;

    // if (GetAccelVals(accel))
    // {
    //     if (GetLinearAccelVals(linAccel))
    //     {
    //         gravity.x = accel.x - linAccel.x;
    //         gravity.y = accel.y - linAccel.y;
    //         gravity.z = accel.z - linAccel.z;
    //         return true;
    //     }
    // }

    // return false;

    


    // DirectionalValues linAccel;

    // bool success = false;

    // sh2_SensorValue_t sensorValue;
    // uint32_t startTime = millis();

    // while (millis() < startTime + 1500)
    // {
    //     if (imu->getSensorEvent(&sensorValue)) 
    //     {
    //         if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION)
    //         {
    //             gravity.x = sensorValue.un.linearAcceleration.x;
    //             gravity.y = sensorValue.un.linearAcceleration.y;
    //             gravity.z = sensorValue.un.linearAcceleration.z;
                
    //             success = true;
    //         }
    //     }
    // }

    // if (!success)
    // {
    //     Serial.println("fail first");
    //     return false;
    // }
    // Serial.println("first done");

    // sh2_SensorValue_t sensorValue2;
    // startTime = millis();

    // return true;

    // while (millis() < startTime + 1500)
    // {
    //     if (imu->getSensorEvent(&sensorValue2)) 
    //     {
    //         if (sensorValue2.sensorId == SH2_ACCELEROMETER)
    //         {
    //             gravity.x = linAccel.x - sensorValue2.un.accelerometer.x;
    //             gravity.y = linAccel.y - sensorValue2.un.accelerometer.y;
    //             gravity.z = linAccel.z - sensorValue2.un.accelerometer.z;
                

                
    //             return true;
    //         }
    //     }
    // }

    // return false;
}


/**
 * @brief Prints out the passed in magnetometer values
 * 
 * @param magn the struct to be populated with magnetometer data
 */
void IMU_BNO085::PrintReadingsMagnetometer(DirectionalValues &magn)
{
    Serial.print("Mag: ");
    Serial.print(magn.x);
    Serial.print(", ");
    Serial.print(magn.y);
    Serial.print(", ");
    Serial.println(magn.z);
}


/**
 * @brief Returns the net acceleration read by the device
 * 
 * @param netAccel the net acceleration of the sensor
 */
float IMU_BNO085::GetNetAcceleration(DirectionalValues& accel)
{
    return sqrt(pow(accel.x, 2) + pow(accel.y, 2) + pow(accel.z, 2));
}


/**
 * @brief Prints the accel values to the terminal
 *
 * @param accel DirectionalValues struct that contains the readings to print
 */
void IMU_BNO085::PrintReadingsAccel(DirectionalValues &accel)
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
void IMU_BNO085::PrintReadingsGyro(DirectionalValues &gyro)
{
    Serial.print("Gyro: ");
    Serial.print(gyro.x);
    Serial.print(", ");
    Serial.print(gyro.y);
    Serial.print(", ");
    Serial.println(gyro.z);
}


/**
 * @brief Prints the quaternion
 *
 * @param quaternion
 */
void IMU_BNO085::PrintQuaternion(Quaternion& quaternion)
{
    // Serial.print("Quaternion (r, i, j, k):  ");
    // Serial.print(quaternion.real);
    // Serial.print(", ");
    // Serial.print(quaternion.i);
    // Serial.print(", ");
    // Serial.print(quaternion.j);
    // Serial.print(", ");
    // Serial.println(quaternion.k);

    Serial.print(quaternion.real);
    Serial.print("/");
    Serial.print(quaternion.i);
    Serial.print("/");
    Serial.print(quaternion.j);
    Serial.print("/");
    Serial.println(quaternion.k);
}