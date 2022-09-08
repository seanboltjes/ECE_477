#ifndef DS_H
#define DS_H
#include <math.h>


class DataStructures
{    
public:
    /**
     * @brief A struct containing directional readings from the IMU.
     * Gryo values are degrees/sec
     * accel values are in g's
     * magnetometer values are in uT
     */
    struct DirectionalValues {
        float x;
        float y;
        float z;
    };

    /**
     * @brief A struct containing a quaternion
     * real: sometimes denoted W.
     * i: sometimes denoted X.
     * j: sometimes denoted Y.
     * k: sometimes denoted Z.
     */
    struct Quaternion
    {
        float real; // sometimes denoted W
        float i;    // sometimes denoted X
        float j;    // sometimes denoted Y
        float k;    // sometimes denoted Z
    };

    /**
     * @brief A struct containing Euler Rotations
     */
    struct EulerRotations
    {
        float roll;
        float pitch;
        float yaw;
    };

};
#endif