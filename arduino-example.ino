#include <Wire.h>
#include "AngleSensor.h"

void setup() {
	Serial.begin(115200);
	Wire.begin();
	
	MPU6050 mpu;
	Quaternion q;
	VectorFloat g;
	
	angleSensor = new AngleSensor(&mpu, &q, &g);
	angleSensor->calibrate();
}

void loop() {
    Serial.println(angleSensor->getAngle());
}
