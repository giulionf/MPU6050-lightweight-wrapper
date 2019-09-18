#ifndef ANGLESENSOR_H
#define ANGLESENSOR_H

#define MPU_PACKET_SIZE 42
#define MPU_DRIFT_THRESHOLD 0.0005
#define ROTATION_DEGREE_TOLERANCE 0.5F

#include <Arduino.h>
#include <MPU6050_6Axis_MotionApps20.h>

class AngleSensor {

public:
		AngleSensor(MPU6050 *m, Quaternion *q, VectorFloat *g);
		~AngleSensor();
		float getAngle();
		bool hasAngle(float wantedAngle);
		float getAngularDistance(float x, float y);
		float angleToRange(float angle);
		float getAnglesPerSecond();
private:
		void calibrate();
		float degreeOffset;
		MPU6050 *mpu;
		Quaternion *quaternion;
		VectorFloat *gravity;
		float ypr[3];
		uint16_t packetSize;
		uint16_t fifoCount;
		uint16_t mpuIntStatus;
		uint8_t fifoBuffer[64];
		int16_t *rpsX; // Rotation Per Second = RPS
		int16_t *rpsY;
		int16_t *rpsZ;
};

#endif


