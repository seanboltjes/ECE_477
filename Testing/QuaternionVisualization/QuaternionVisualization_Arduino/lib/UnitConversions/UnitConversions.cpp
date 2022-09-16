#include "UnitConversions.h"



/**
 * @brief Converts celsius into fahrenheit
 * 
 * @param celsius
 * 
 * @return float converted to fahrenheit
 */
float UnitConversions::CelsiusToFahrenheit(float celsius)
{
    return celsius * 1.8 + 32;
}


/**
 * @brief Converts fahrenheit into celcius
 * 
 * @param fahrenheit
 * 
 * @return float converted to celsius
 */
float UnitConversions::FahrenheitToCelsius(float fahrenheit)
{
    return (fahrenheit - 32) * (5 / 9);
}


/**
 * @brief Converts meters into feet
 * 
 * @param meters
 * 
 * @return float converted to feet
 */
float UnitConversions::MetersToFeet(float meters)
{
    return meters * 3.28084;
}


/**
 * @brief Converts feet into meters
 * 
 * @param feet
 * 
 * @return float converted to meters
 */
float UnitConversions::FeetToMeters(float feet)
{
    return feet / 3.28084;
}


/**
 * @brief Converts radians into degrees
 * 
 * @param radians
 * 
 * @return float converted to degrees
 */
float UnitConversions::RadiansToDegrees(float radians)
{
    return radians * 180 / DEFINED_PI;
}


/**
 * @brief Converts degrees into radians
 * 
 * @param degrees
 * 
 * @return float converted to radians
 */
float UnitConversions::DegreesToRadians(float degrees)
{
    return degrees * DEFINED_PI / 180;
}


/**
 * @brief Converts meters per second squared (m/s/s) to gravity g's
 * 
 * @param metersPerSecondSquared
 * 
 * @return float converted to g's
 */
float UnitConversions::MetersPerSecondSquaredToGs(float metersPerSecondSquared)
{
    return (metersPerSecondSquared * 0.101971621);
}


/**
 * @brief Converts gravity g's to meters per second squared (m/s/s)
 * 
 * @param metersPerSecondSquared
 * 
 * @return float converted to meters per second squared (m/s/s)
 */
float UnitConversions::GsToMetersPerSecondSquared(float gs)
{
    return (gs * 9.8066500286);
}