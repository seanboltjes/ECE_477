#ifndef UNIT_CONVERSIONS_H
#define UNIT_CONVERSIONS_H
#include <math.h>

#define DEFINED_PI 3.1415926536

class UnitConversions
{
public:
    static float CelsiusToFahrenheit(float celsius);
    static float FahrenheitToCelsius(float fahrenheit);
    static float MetersToFeet(float meters);
    static float FeetToMeters(float feet);
    static float RadiansToDegrees(float radians);
    static float DegreesToRadians(float degrees);

private:

};


 #endif