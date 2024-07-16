#include "bno08x.h"

#include <stdlib.h>
#include <stdio.h>
#include "sh2.h"
#include "sh2_err.h"
#include "sh2_util.h"
#include "sh2_SensorValue.h"

bool reset_occurred = false;

static void event_handler(void *cookie, sh2_AsyncEvent_t *pEvent) {
	printf("event_handler\n");
	// If we see a reset, set a flag so that sensors will be reconfigured.
	if (pEvent->eventId == SH2_RESET) {
		reset_occurred = true;
	} else if (pEvent->eventId == SH2_SHTP_EVENT) {
		printf("EventHandler  id:SHTP, %d\n", pEvent->shtpEvent);
	} else if (pEvent->eventId == SH2_GET_FEATURE_RESP) {
		printf("EventHandler Sensor Config, %d\n",
				pEvent->sh2SensorConfigResp.sensorId);
	} else {
		printf("EventHandler, unknown event Id: %ld\n", pEvent->eventId);
	}
}

// Configure one sensor to produce periodic reports
static void startReports() {
	int status;

	// Each entry of sensorConfig[] represents one sensor to be configured in the loop below
	static const struct {
		int sensorId;
		sh2_SensorConfig_t config;
	} sensorConfig[] = {
	// Game Rotation Vector, 100Hz
			{ SH2_GAME_ROTATION_VECTOR, { .reportInterval_us = 1000 * 8 } },

	// Stability Detector, 100 Hz, changeSensitivityEnabled
	// {SH2_STABILITY_DETECTOR, {.reportInterval_us = 10000, .changeSensitivityEnabled = true}},

	// Raw accel, 100 Hz
	// {SH2_RAW_ACCELEROMETER, {.reportInterval_us = 10000}},

	// Raw gyroscope, 100 Hz
	// {SH2_RAW_GYROSCOPE, {.reportInterval_us = 10000}},

	// Rotation Vector, 100 Hz
	// {SH2_ROTATION_VECTOR, {.reportInterval_us = 10000}},

	// Gyro Integrated Rotation Vector, 100 Hz
	// {SH2_GYRO_INTEGRATED_RV, {.reportInterval_us = 10000}},

	// Motion requests for Interactive Zero Reference Offset cal
	// {SH2_IZRO_MOTION_REQUEST, {.reportInterval_us = 10000}},

	// Shake detector
	// {SH2_SHAKE_DETECTOR, {.reportInterval_us = 10000}},
			};

	for (int n = 0; n < ARRAY_LEN(sensorConfig); n++) {
		int sensorId = sensorConfig[n].sensorId;

		status = sh2_setSensorConfig(sensorId, &sensorConfig[n].config);
		if (status != 0) {
			printf("Error while enabling sensor %d\n", sensorId);
		}
	}
}

static void sensor_handler(void *cookie, sh2_SensorEvent_t *pEvent) {
	float t, r, i, j, k, acc_rad;
	float angVelX, angVelY, angVelZ;
	static uint32_t lastSequence[SH2_MAX_SENSOR_ID + 1]; // last sequence number for each sensor
	sh2_SensorValue_t value;

	// Convert event to value
	sh2_decodeSensorEvent(&value, pEvent);

	// Compute new sample_id
	uint8_t deltaSeq = value.sequence - (lastSequence[value.sensorId] & 0xFF);
	lastSequence[value.sensorId] += deltaSeq;

	// Get time as float
	t = value.timestamp / 1000000.0;

	switch (value.sensorId) {
	case SH2_RAW_ACCELEROMETER:
		printf(".%d %0.6f, %d, %d, %d, %d\n", SH2_RAW_ACCELEROMETER, t,
				lastSequence[value.sensorId], value.un.rawAccelerometer.x,
				value.un.rawAccelerometer.y, value.un.rawAccelerometer.z);
		break;

	case SH2_RAW_MAGNETOMETER:
		printf(".%d %0.6f, %d, %d, %d, %d\n", SH2_RAW_MAGNETOMETER, t,
				lastSequence[value.sensorId], value.un.rawMagnetometer.x,
				value.un.rawMagnetometer.y, value.un.rawMagnetometer.z);
		break;

	case SH2_RAW_GYROSCOPE:
		printf(".%d %0.6f, %d, %d, %d, %d\n", SH2_RAW_GYROSCOPE, t,
				lastSequence[value.sensorId], value.un.rawGyroscope.x,
				value.un.rawGyroscope.y, value.un.rawGyroscope.z);
		break;

	case SH2_MAGNETIC_FIELD_CALIBRATED:
		printf(".%d %0.6f, %d, %0.6f, %0.6f, %0.6f, %u\n",
				SH2_MAGNETIC_FIELD_CALIBRATED, t, lastSequence[value.sensorId],
				value.un.magneticField.x, value.un.magneticField.y,
				value.un.magneticField.z, value.status & 0x3);
		break;

	case SH2_ACCELEROMETER:
		printf(".%d %0.6f, %d, %0.6f, %0.6f, %0.6f\n", SH2_ACCELEROMETER, t,
				lastSequence[value.sensorId], value.un.accelerometer.x,
				value.un.accelerometer.y, value.un.accelerometer.z);
		break;

	case SH2_ROTATION_VECTOR:
		r = value.un.rotationVector.real;
		i = value.un.rotationVector.i;
		j = value.un.rotationVector.j;
		k = value.un.rotationVector.k;
		acc_rad = value.un.rotationVector.accuracy;
		printf(".%d %0.6f, %d, %0.6f, %0.6f, %0.6f, %0.6f, %0.6f\n",
				SH2_ROTATION_VECTOR, t, lastSequence[value.sensorId], r, i, j,
				k, acc_rad);
		break;

	case SH2_GAME_ROTATION_VECTOR:
		r = value.un.gameRotationVector.real;
		i = value.un.gameRotationVector.i;
		j = value.un.gameRotationVector.j;
		k = value.un.gameRotationVector.k;
		printf(".%d %0.6f, %d, %0.6f, %0.6f, %0.6f, %0.6f\n",
				SH2_GAME_ROTATION_VECTOR, t, lastSequence[value.sensorId], r, i,
				j, k);
		break;

	case SH2_GYRO_INTEGRATED_RV:
		angVelX = value.un.gyroIntegratedRV.angVelX;
		angVelY = value.un.gyroIntegratedRV.angVelY;
		angVelZ = value.un.gyroIntegratedRV.angVelZ;
		r = value.un.gyroIntegratedRV.real;
		i = value.un.gyroIntegratedRV.i;
		j = value.un.gyroIntegratedRV.j;
		k = value.un.gyroIntegratedRV.k;
		printf(".%d %0.6f, %0.6f, %0.6f, %0.6f, %0.6f, %0.6f, %0.6f, %0.6f\n",
				SH2_GYRO_INTEGRATED_RV, t, angVelX, angVelY, angVelZ, r, i, j,
				k);
		break;
	default:
		printf("Unknown sensor: %d\n", value.sensorId);
		break;
	}
}

void sensor_handler_wrapper(void *cookie, sh2_SensorEvent_t *pEvent) {
	float t, r, i, j, k, acc_rad;
	float angVelX, angVelY, angVelZ;
	static uint32_t lastSequence[SH2_MAX_SENSOR_ID + 1]; // last sequence number for each sensor
	sh2_SensorValue_t value;

	// Convert event to value
	sh2_decodeSensorEvent(&value, pEvent);

	// Compute new sample_id
	uint8_t deltaSeq = value.sequence - (lastSequence[value.sensorId] & 0xFF);
	lastSequence[value.sensorId] += deltaSeq;

	// Get time as float
	t = value.timestamp / 1000000.0;

	bno08x_sensorType_t sensorType = UNKNOWN;
	bno08x_data_t sensorData;

	sensorData.timestamp_uS = value.timestamp;

	switch (value.sensorId) {
	case SH2_RAW_ACCELEROMETER:
		// printf(".%d %0.6f, %d, %d, %d, %d\n",
		//        SH2_RAW_ACCELEROMETER,
		//        t,
		//        lastSequence[value.sensorId],
		//        value.un.rawAccelerometer.x,
		//        value.un.rawAccelerometer.y,
		//        value.un.rawAccelerometer.z);
		sensorType = BNO08X_RAW_ACCELEROMETER;
		sensorData.raw_accelerometer.x = value.un.rawAccelerometer.x;
		sensorData.raw_accelerometer.y = value.un.rawAccelerometer.y;
		sensorData.raw_accelerometer.z = value.un.rawAccelerometer.z;
		break;

	case SH2_RAW_MAGNETOMETER:
		// printf(".%d %0.6f, %d, %d, %d, %d\n",
		//        SH2_RAW_MAGNETOMETER,
		//        t,
		//        lastSequence[value.sensorId],
		//        value.un.rawMagnetometer.x,
		//        value.un.rawMagnetometer.y,
		//        value.un.rawMagnetometer.z);
		sensorType = BNO08X_RAW_MAGNETOMETER;
		sensorData.raw_magnetometer.x = value.un.rawMagnetometer.x;
		sensorData.raw_magnetometer.y = value.un.rawMagnetometer.y;
		sensorData.raw_magnetometer.z = value.un.rawMagnetometer.z;
		break;

	case SH2_RAW_GYROSCOPE:
		// printf(".%d %0.6f, %d, %d, %d, %d\n",
		//        SH2_RAW_GYROSCOPE,
		//        t,
		//        lastSequence[value.sensorId],
		//        value.un.rawGyroscope.x,
		//        value.un.rawGyroscope.y,
		//        value.un.rawGyroscope.z);
		sensorType = BNO08X_RAW_GYROSCOPE;
		sensorData.raw_gyroscope.x = value.un.rawGyroscope.x;
		sensorData.raw_gyroscope.y = value.un.rawGyroscope.y;
		sensorData.raw_gyroscope.z = value.un.rawGyroscope.z;
		break;

	case SH2_MAGNETIC_FIELD_CALIBRATED:
		// printf(".%d %0.6f, %d, %0.6f, %0.6f, %0.6f, %u\n",
		//        SH2_MAGNETIC_FIELD_CALIBRATED,
		//        t,
		//        lastSequence[value.sensorId],
		//        value.un.magneticField.x,
		//        value.un.magneticField.y,
		//        value.un.magneticField.z,
		//        value.status & 0x3);
		sensorType = BNO08X_MAGNETIC_FIELD;
		sensorData.magnetic_field.x = value.un.magneticField.x;
		sensorData.magnetic_field.y = value.un.magneticField.y;
		sensorData.magnetic_field.z = value.un.magneticField.z;
		break;

	case SH2_ACCELEROMETER:
		// printf(".%d %0.6f, %d, %0.6f, %0.6f, %0.6f\n",
		//        SH2_ACCELEROMETER,
		//        t,
		//        lastSequence[value.sensorId],
		//        value.un.accelerometer.x,
		//        value.un.accelerometer.y,
		//        value.un.accelerometer.z);
		sensorType = BNO08X_ACCELEROMETER;
		sensorData.accelerometer.x = value.un.accelerometer.x;
		sensorData.accelerometer.y = value.un.accelerometer.y;
		sensorData.accelerometer.z = value.un.accelerometer.z;
		break;

	case SH2_ROTATION_VECTOR:
		// r = value.un.rotationVector.real;
		// i = value.un.rotationVector.i;
		// j = value.un.rotationVector.j;
		// k = value.un.rotationVector.k;
		// acc_rad = value.un.rotationVector.accuracy;
		// printf(".%d %0.6f, %d, %0.6f, %0.6f, %0.6f, %0.6f, %0.6f\n",
		//        SH2_ROTATION_VECTOR,
		//        t,
		//        lastSequence[value.sensorId],
		//        r, i, j, k,
		//        acc_rad);
		sensorType = BNO08X_ROTATION_VECTOR;
		sensorData.rotationVector.i = value.un.rotationVector.i;
		sensorData.rotationVector.j = value.un.rotationVector.j;
		sensorData.rotationVector.k = value.un.rotationVector.k;
		sensorData.rotationVector.real = value.un.rotationVector.real;
		sensorData.rotationVector.accuracy = value.un.rotationVector.accuracy;
		break;

	case SH2_GAME_ROTATION_VECTOR:
		// r = value.un.gameRotationVector.real;
		// i = value.un.gameRotationVector.i;
		// j = value.un.gameRotationVector.j;
		// k = value.un.gameRotationVector.k;
		// printf(".%d %0.6f, %d, %0.6f, %0.6f, %0.6f, %0.6f\n",
		//        SH2_GAME_ROTATION_VECTOR,
		//        t,
		//        lastSequence[value.sensorId],
		//        r, i, j, k);
		sensorType = BNO08X_ROTATION_VECTOR;
		sensorData.gameRotationVector.i = value.un.gameRotationVector.i;
		sensorData.gameRotationVector.j = value.un.gameRotationVector.j;
		sensorData.gameRotationVector.k = value.un.gameRotationVector.k;
		sensorData.gameRotationVector.real = value.un.gameRotationVector.real;
		break;

	case SH2_GYRO_INTEGRATED_RV:
		// angVelX = value.un.gyroIntegratedRV.angVelX;
		// angVelY = value.un.gyroIntegratedRV.angVelY;
		// angVelZ = value.un.gyroIntegratedRV.angVelZ;
		// r = value.un.gyroIntegratedRV.real;
		// i = value.un.gyroIntegratedRV.i;
		// j = value.un.gyroIntegratedRV.j;
		// k = value.un.gyroIntegratedRV.k;
		// printf(".%d %0.6f, %0.6f, %0.6f, %0.6f, %0.6f, %0.6f, %0.6f, %0.6f\n",
		//        SH2_GYRO_INTEGRATED_RV,
		//        t,
		//        angVelX, angVelY, angVelZ,
		//        r, i, j, k);
		sensorType = BNO08X_GYRO_INTEGRATED_RV;
		sensorData.gyroIntegratedRV.i = value.un.gyroIntegratedRV.i;
		sensorData.gyroIntegratedRV.j = value.un.gyroIntegratedRV.j;
		sensorData.gyroIntegratedRV.k = value.un.gyroIntegratedRV.k;
		sensorData.gyroIntegratedRV.real = value.un.gyroIntegratedRV.real;
		sensorData.gyroIntegratedRV.angVelX = value.un.gyroIntegratedRV.angVelX;
		sensorData.gyroIntegratedRV.angVelY = value.un.gyroIntegratedRV.angVelY;
		sensorData.gyroIntegratedRV.angVelZ = value.un.gyroIntegratedRV.angVelZ;
		break;
	default:
		printf("Unknown sensor: %d\n", value.sensorId);
		break;
	}

	bno08x_handler_t *callback = (bno08x_handler_t*) cookie;
	callback(&sensorData);
}

void bno08x_open(void *hal, bno08x_handler_t *callback) {
//	int status = sh2_open(hal, event_handler, NULL);
	int status = sh2_open(hal, NULL, NULL);
	printf("sh2_open\n");
	if (status != SH2_OK) {
		printf("sh2_open ERROR: %d\n", status);
	}
	sh2_setSensorCallback(sensor_handler_wrapper, callback);
//	sh2_setSensorCallback(sensor_handler, NULL);
	reset_occurred = false;
	startReports();
}

void bno08x_update() {
	if (reset_occurred) {
		printf("reset occurred");
		// Restart the flow of sensor reports
		reset_occurred = false;
		startReports();
	}

	sh2_service();
}
