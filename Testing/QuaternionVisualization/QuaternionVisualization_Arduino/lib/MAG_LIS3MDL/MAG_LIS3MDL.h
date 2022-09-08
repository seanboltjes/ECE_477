#ifndef MAG_LIS3MDL_H
#define MAG_LIS3MDL_H
#include <math.h>
#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include "../DataStructures/DataStructures.h"


/**
 * @brief class for the LIS3MDL 3-axis magnetometer
 */
class MAG_LIS3MDL : public DataStructures
{   
public:
    virtual bool GetMagnetometerVals(DirectionalValues& magn);
    static void PrintReadingsMagnetometer(DirectionalValues& magn);

protected:
    virtual void InitCompass();
    virtual ~MAG_LIS3MDL();

    Adafruit_LIS3MDL* compass;
};

#endif