#include "IMU_9DOF.h"


IMU_9DOF::IMU_9DOF()
{
    filter = new Adafruit_Mahony();
    filter->begin(101);    // 48  50  29
}

bool IMU_9DOF::GetQuaternion(Quaternion& quaternion)
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

bool IMU_9DOF::GetGravityVector(DirectionalValues& gravityVector, DirectionalValues accel, DirectionalValues gyro, DirectionalValues magnet)
{
    // float x, y, z;

    // filter->getGravityVector(&x, &y, &z);

    // gravityVector.x = x;
    // gravityVector.y = y;
    // gravityVector.z = z;


    
    Euler eulerRotation;
    GetEulerRotation(eulerRotation, accel, gyro, magnet);

    gravityVector.z = cos(UnitConversions::DegreesToRadians(eulerRotation.roll)) * cos(UnitConversions::DegreesToRadians(eulerRotation.pitch));
    gravityVector.y = sin(UnitConversions::DegreesToRadians(eulerRotation.roll)) * cos(UnitConversions::DegreesToRadians(eulerRotation.pitch));
    gravityVector.x = -1 * sin(UnitConversions::DegreesToRadians(eulerRotation.pitch));

    
    gravX = gravityVector.x;
    gravY = gravityVector.y;
    gravZ = gravityVector.z;

    return true;
}

void IMU_9DOF::CorrectAccel(DirectionalValues& accel)
{
    accel.x -= gravX;
    accel.y -= gravY;
    accel.z -= gravZ;
}


void IMU_9DOF::UpdatePosition(DirectionalValues& correctedAccel, uint32_t timeSinceLastUpdate_us, bool& newMeasureReady)
{
    double accelTermX, accelTermY, accelTermZ;

    if (correctedAccel.x < 0.03 && correctedAccel.x > -0.03)
    {
        correctedAccel.x = 0.0f;
        numZeroX++;
    }
    if (correctedAccel.y < 0.03 && correctedAccel.y > -0.03)
    {
        correctedAccel.y = 0.0f;
        numZeroY++;
    }
    if (correctedAccel.z < 0.03 && correctedAccel.z > -0.03)
    {
        correctedAccel.z = 0.0f;
        numZeroZ++;
    }
    

    if (numZeroX >= 3)
    {
        posXCache = posX;
        numZeroX = 0;
        velX = 0.0f;
        posX = 0.0f;
        newMeasureReady = true;
    }
    if (numZeroY >= 3)
    {
        posYCache = posY;
        numZeroY = 0;
        velY = 0.0f;
        posY = 0.0f;
        newMeasureReady = true;
    }
    if (numZeroZ >= 3)
    {
        posZCache = posZ;
        numZeroZ = 0;
        velZ = 0.0f;
        posZ = 0.0f;
        newMeasureReady = true;
    }

    double timeSinceLastUpdate_us_squared = pow(timeSinceLastUpdate_us, 2);
    double timeSinceLastUpdate_sec = timeSinceLastUpdate_us * 0.000001;
    double timeSinceLastUpdate_sec_squared = timeSinceLastUpdate_us_squared * 0.000001;
    double accelTermCoeff = timeSinceLastUpdate_sec_squared * 0.5;

    velX += timeSinceLastUpdate_sec * correctedAccel.x;
    velY += timeSinceLastUpdate_sec * correctedAccel.y;
    velZ += timeSinceLastUpdate_sec * correctedAccel.z;

    accelTermX = correctedAccel.x * accelTermCoeff;
    accelTermY = correctedAccel.y * accelTermCoeff;
    accelTermZ = correctedAccel.z * accelTermCoeff;

    posX += (velX * timeSinceLastUpdate_sec + accelTermX);
    posY += (velY * timeSinceLastUpdate_sec + accelTermY);
    posZ += (velZ * timeSinceLastUpdate_sec + accelTermZ);
}


bool IMU_9DOF::GetEulerRotation(Euler& eulerRotations, DirectionalValues accel, DirectionalValues gyro, DirectionalValues magnet)
{
    Quaternion quaternion;
    if (GetQuaternion(quaternion, accel, gyro, magnet))
    {
        float t0 = 2.0 * (quaternion.real * quaternion.i + quaternion.j * quaternion.k);
        float t1 = 1.0 - 2.0 * (quaternion.i * quaternion.i + quaternion.j * quaternion.j);
        eulerRotations.roll = UnitConversions::RadiansToDegrees(atan2(t0, t1));

        float t2 = +2.0 * (quaternion.real * quaternion.j - quaternion.k * quaternion.i);
        
        t2 = t2 > 1 ? 1.0 : t2;
        t2 = t2 < -1 ? -1.0 : t2;
        
        eulerRotations.pitch = UnitConversions::RadiansToDegrees(asin(t2));

        float t3 = +2.0 * (quaternion.real * quaternion.k + quaternion.i * quaternion.j);
        float t4 = +1.0 - 2.0 * (quaternion.j * quaternion.j + quaternion.k * quaternion.k);
        eulerRotations.yaw = UnitConversions::RadiansToDegrees(atan2(t3, t4));

        return true;
    }

    return false;
}


bool IMU_9DOF::GetQuaternion(Quaternion& quaternion, DirectionalValues accel, DirectionalValues gyro, DirectionalValues magnet)
{
    filter->update(gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z, magnet.x, magnet.y, magnet.z);

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


void IMU_9DOF::ComputeQuaternion(Quaternion& quaternion, DirectionalValues& accel, DirectionalValues& gyro, DirectionalValues& magnet)
{
    const double beta = 0.6045998;

    // float q1 = quaternion.real, q2 = quaternion.i, q3 = quaternion.j, q4 = quaternion.k;   // short name local variable for readability
    float q1 = 1.0f, q2 = 0.0f, q3 = 0.0f, q4 = 0.0f;   // short name local variable for readability
    float ax = accel.x;
    float ay = accel.y;
    float az = accel.z;
    float gx = gyro.x * PI / 180.0f;
    float gy = gyro.y * PI / 180.0f;
    float gz = gyro.z * PI / 180.0f;
    float mx = magnet.x;
    float my = magnet.y;
    float mz = magnet.z;

    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrtf(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * mx;
    _2q1my = 2.0f * q1 * my;
    _2q1mz = 2.0f * q1 * mz;
    _2q2mx = 2.0f * q2 * mx;
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
    norm = 1.0f/norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * deltat;
    q2 += qDot2 * deltat;
    q3 += qDot3 * deltat;
    q4 += qDot4 * deltat;
    norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;

    quaternion.real = q1 * norm;
    quaternion.i    = q2 * norm;
    quaternion.j    = q3 * norm;
    quaternion.k    = q4 * norm;
}


void IMU_9DOF::PrintReadingsMagnetometer(DirectionalValues &magn)
{
    Serial.print("Mag: ");
    Serial.print(magn.x);
    Serial.print(", ");
    Serial.print(magn.y);
    Serial.print(", ");
    Serial.println(magn.z);
}


void IMU_9DOF::GetAndPrintAllReadings()
{
    DirectionalValues accel;
    DirectionalValues gyro;
    DirectionalValues magnetometer;
    
    GetAccelVals(accel);
    GetGyroVals(gyro);
    GetMagnetometerVals(magnetometer);
    
    Serial.print(accel.x);
    Serial.print("/");
    Serial.print(accel.y);
    Serial.print("/");
    Serial.print(accel.z);
    Serial.print("/");
    Serial.print(gyro.x);
    Serial.print("/");
    Serial.print(gyro.y);
    Serial.print("/");
    Serial.print(gyro.z);
    Serial.print("/");
    Serial.print(magnetometer.x);
    Serial.print("/");
    Serial.print(magnetometer.y);
    Serial.print("/");
    Serial.println(magnetometer.z);
}


void IMU_9DOF::PrintQuaternion(Quaternion& quaternion)
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


void IMU_9DOF::PrintEulerRotations(Euler& eulerRotations)
{
    Serial.print(eulerRotations.roll);
    Serial.print("/");
    Serial.print(eulerRotations.pitch);
    Serial.print("/");
    Serial.println(eulerRotations.yaw);
}


void IMU_9DOF::PrintCurrentPosition(DirectionalValues& accel)
{
    Serial.print("Position(m) (x, y, z):  (");
    Serial.print(posXCache);
    Serial.print(", ");
    Serial.print(posYCache);
    Serial.print(", ");
    Serial.print(posZCache);
    Serial.print(")   Corrected Accel(m/s/s) (x, y, z):  ");

    Serial.print(accel.x);
    Serial.print(", ");
    Serial.print(accel.y);
    Serial.print(", ");
    Serial.println(accel.z);
}