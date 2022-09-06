# What is the `IMU_6DOF` abstract class?
This class is an `abstract` class, meaning that is kind of like a template. It isn't exactly a template, because that's a specific type of class in C++ but we're using it as a template for other IMU_6DOF modules.
Every single function, struct, variable, etc in this class will be available in its child classes.

Use this class for any **I**nertial **M**easurement **U**nit with **6** **D**egrees **O**f **F**reedom.

# When should I use this class?
## Having functions that need an IMU_6DOF
Any function that needs a IMU_6DOF module should take this `IMU_6DOF` class as a parameter.


For example, let's say you have a method called `bool IsPayloadStationary(IMU_6DOF imu)`.

Notice here I have the parameter type as `IMU_6DOF`. This means any child class of `IMU_6DOF` can be passed in here. We could pass in an object of `IMU_6DOF_LSM6x`, `IMU_6DOF_MPU6500`, etc as long as the class that gets passed in is a child class of `IMU_6DOF`.

Example:
```C++
foo()
{
  // we make an IMU_6DOF_LSM6x object (a child of IMU_6DOF)
  IMU_6DOF_LSM6x imu;

  ...
  
  // we call IsPayloadStationary (notice that we pass in a IMU_6DOF_LSM6x, NOT an IMU_6DOF object)
  IsPayloadStationary(imu);
  
  ...
}
```

## Making child classes
Whenever you have a new IMU_6DOF module, have the child class you make for it inherit this class.

For example, let's say I'm making a new child class for the MPU6500 IMU module.
1. Start by creating a new child class called `IMU_6DOF_MPU6500`.
2. Have it inherit this class, like so: `class IMU_6DOF_MPU6500 : public IMU_6DOF`.

Any abstract function in the `IMU_6DOF` class needs to implemented in the `IMU_6DOF_MPU6500` class. You can find these by looking at all the fuctions of this general format in the header file `virtual void foo() = 0;`

Now your child class can be used in the control loop while changing virtually nothing because its functionality is the same as other IMU_6DOF classes.


# Example Test Code (change pins and such accordingly)
```C++
#include <Arduino.h>
#include childClass

// TODO: change childClass to the respective child class, ex: IMU_6DOF_LSM6x

childClass imu;

void setup()
{
    Serial.begin(9600);
    imu.Init();
}


void loop()
{
    IMU_6DOF::DirectionalValues accel;
    IMU_6DOF::DirectionalValues gyro;
    
    if (imu.GetAccelVals(accel))
        IMU_6DOF::PrintReadingsAccel(accel);
        
    if (imu.GetGyroVals(gyro))
        IMU_6DOF::PrintReadingsGyro(gyro);
    
    delay(400);
}

```
