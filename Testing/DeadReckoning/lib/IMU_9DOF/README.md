# What is the `IMU_9DOF` abstract class?
This class is an `abstract` class, meaning that is kind of like a template. It isn't exactly a template, because that's a specific type of class in C++ but we're using it as a template for other IMU_9DOF modules.
Every single function, struct, variable, etc in this class will be available in its child classes.

Use this class for any **I**nertial **M**easurement **U**nit with **9** **D**egrees **O**f **F**reedom. 

*Note that because this inherits from IMU_6DOF, wherever an IMU_6DOF is required, an IMU_9DOF can be passed in instead. 

# When should I use this class?
## Having functions that need an IMU_9DOF
Any function that needs a IMU_9DOF module should take this `IMU_9DOF` class as a parameter.


For example, let's say you have a method called `bool IsPayloadStationary(IMU_9DOF imu)`.

Notice here I have the parameter type as `IMU_9DOF`. This means any child class of `IMU_6DOF` can be passed in here. We could pass in an object of `IMU_9DOF_BNO08x`, `IMU_9DOF_MPU6212`, etc as long as the class that gets passed in is a child class of `IMU_9DOF`.

Example:
```C++
foo()
{
  // we make an IMU_9DOF_BNO08x object (a child of IMU_9DOF)
  IMU_9DOF_BNO08x imu;

  ...
  
  // we call IsPayloadStationary (notice that we pass in a IMU_9DOF_BNO08x, NOT an IMU_9DOF object)
  IsPayloadStationary(imu);
  
  ...
}
```

## Making child classes
Whenever you have a new IMU_9DOF module, have the child class you make for it inherit this class.

For example, let's say I'm making a new child class for the MPU6212 IMU module.
1. Start by creating a new child class called `IMU_9DOF_MPU6212`.
2. Have it inherit this class, like so: `class IMU_9DOF_MPU6212 : public IMU_9DOF`.

Any abstract function in the `IMU_9DOF` class needs to implemented in the `IMU_9DOF_MPU6212` class. You can find these by looking at all the fuctions of this general format in the header file `virtual void foo() = 0;`

Now your child class can be used in the control loop while changing virtually nothing because its functionality is the same as other IMU_6DOF classes.


# Example Test Code (change pins and such accordingly)
```C++
#include <Arduino.h>
#include childClass

// TODO: change childClass to the respective child class, ex: IMU_9DOF_BNO08x

childClass imu;

void setup()
{
    Serial.begin(9600);
    imu.Init();
}


void loop()
{
    IMU_9DOF::DirectionalValues accel;
    IMU_9DOF::DirectionalValues gyro;
    IMU_9DOF::DirectionalValues magn;
    
    if (imu.GetAccelVals(accel))
        IMU_9DOF::PrintReadingsAccel(accel);
        
    if (imu.GetGyroVals(gyro))
        IMU_9DOF::PrintReadingsGyro(gyro);
        
      if (imu.GetMagnetometerVals(magn))
        IMU_9DOF::PrintReadingsGyro(magn);
    
    delay(400);
}

```
