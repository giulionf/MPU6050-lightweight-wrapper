#include "AngleSensor.h"

// Initializes the Angle Sensor
AngleSensor::AngleSensor(MPU6050 *m, Quaternion *q, VectorFloat *g){
	
	mpu = m;
	quaternion = q;
	gravity = g;
	
	mpu->initialize();
	mpu->dmpInitialize();
    mpu->setXGyroOffset(29);
    mpu->setYGyroOffset(-37);
    mpu->setZGyroOffset(-45);
    mpu->setZAccelOffset(1243);
	mpu->setDMPEnabled(true);
	packetSize = MPU_PACKET_SIZE;
	calibrate();
}

// Cleanes the Memeory after delete call
AngleSensor::~AngleSensor() {
	delete mpu;
	delete quaternion;
	delete gravity;
	delete ypr;
	delete fifoBuffer;
}

// Measures and returns the current Angle
float AngleSensor::getAngle() {
	mpu->resetFIFO();
	fifoCount = mpu->getFIFOCount();
	mpuIntStatus = mpu->getIntStatus();

	// Data Ready
	if (mpuIntStatus & 0x02) {
	
		while (fifoCount < packetSize) {
			fifoCount = mpu->getFIFOCount();
		}

		mpu->getFIFOBytes(fifoBuffer, packetSize);
		fifoCount -= packetSize;
		mpu->dmpGetQuaternion(quaternion, fifoBuffer);
		mpu->dmpGetGravity(gravity, quaternion);
		mpu->dmpGetYawPitchRoll(ypr, quaternion, gravity); 

		return 180.0 + (ypr[0] * (180.0 / PI)) - degreeOffset;  
	}

	// Try until there is a result
	return getAngle();
}

// Returns if a given Angle has been reached
bool AngleSensor::hasAngle(float wantedAngle) {
    float currentAngle = getAngle();
    return (currentAngle > wantedAngle - ROTATION_DEGREE_TOLERANCE 
				&& currentAngle < wantedAngle + ROTATION_DEGREE_TOLERANCE);
}

// Returns the difference between to angles in degrees
float AngleSensor::getAngularDistance(float x, float y) {
	float a = (x * PI / 180.0) - PI;
    float b = (y * PI / 180.0) - PI;
    return atan2(sin(b - a), cos(b - a)) * (180.0 / PI);
}

// Normalizes an angle so it is between 0 and 360 degrees
float AngleSensor::angleToRange(float angle) {
    // Subtract the angle overflow
    while (angle > 360) {
        angle -= 360.0;
    }

    // Substract "underflow"
    while (angle < 0) {
        angle += 360.0;
    }

    return angle;
}

// Returns the filtered Gyroscope Data
float AngleSensor::getAnglesPerSecond() {
	mpu->getRotation(rpsX, rpsY, rpsZ);
	return (float)(*rpsZ) / 131.0;
}

// Calibrate and stabilize (the gyroscope drift)
void AngleSensor::calibrate() {
	do {
		degreeOffset += getAngle();
		delay(500);
		Serial.print("DEGREE OFFSET: ");
		Serial.println(degreeOffset);
	}
	while (abs(getAngle()) > MPU_DRIFT_THRESHOLD);
	
}