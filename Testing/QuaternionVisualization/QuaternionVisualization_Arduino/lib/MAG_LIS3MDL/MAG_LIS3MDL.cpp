#include "MAG_LIS3MDL.h"




/**
 * @brief Initializes the Magnetometer (a compass)
 */
void MAG_LIS3MDL::InitCompass()
{
    Wire.begin();
    compass = new Adafruit_LIS3MDL();

    while (!compass->begin_I2C()){
        Serial.println("LIS3MDL does not respond");
        delay(100);
    }
    
    compass->setPerformanceMode(LIS3MDL_MEDIUMMODE);
    compass->setOperationMode(LIS3MDL_CONTINUOUSMODE);
    compass->setDataRate(LIS3MDL_DATARATE_560_HZ);
    compass->setRange(LIS3MDL_RANGE_4_GAUSS);
    
    Serial.println("LIS3MDL setup complete");
}


/**
 * @brief Destructor. Frees allocated memory
 */
MAG_LIS3MDL::~MAG_LIS3MDL()
{
    delete compass;
}


/**
 * @brief Returns a struct with the current magnetometer values (in micro Tesla)
 * 
 * @param magn the struct to be populated with magnetometer data
 * 
 * @return bool - true if successful false if not successful
 */
bool MAG_LIS3MDL::GetMagnetometerVals(DirectionalValues& magn)
{
    sensors_event_t event; 
    bool wasSuccess = compass->getEvent(&event);

    magn.x = event.magnetic.x;
    magn.y = event.magnetic.y;
    magn.z = event.magnetic.z;

    return wasSuccess;
}


/**
 * @brief Prints out the passed in magnetometer values
 * 
 * @param magn the struct to be populated with magnetometer data
 */
void MAG_LIS3MDL::PrintReadingsMagnetometer(DirectionalValues &magn)
{
    Serial.print("Mag: ");
    Serial.print(magn.x);
    Serial.print(", ");
    Serial.print(magn.y);
    Serial.print(", ");
    Serial.println(magn.z);
}