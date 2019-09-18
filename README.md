### About

Lightweight Wrapper around the MPU6050 Code provided by [I2Cdevlib](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050 "I2Cdevlib"). 

### Features
- Calibrate the sensor to avoid gyroscope drift
- Read the angle of the sensor via single method call

### Examples
###### Create the Wrapper Object
```cpp
MPU6050 mpu;
Quaternion q;
VectorFloat g;
angleSensor = new AngleSensor(&mpu, &q, &g);
```
###### Calibrate the sensor
```cpp
angleSensor->calibrate();
```
###### Read the Angle
```cpp
angleSensor->getAngle();
```

### Contributions
Contributions are very welcome, please provide an issue and make a pull request to contribute to the project.
