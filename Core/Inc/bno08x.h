#if !defined(BNO08X_H)
#define BNO08X_H

#include <inttypes.h>

typedef struct
{
    /* Units are ADC counts */
    int16_t x; /**< @brief [ADC counts] */
    int16_t y; /**< @brief [ADC counts] */
    int16_t z; /**< @brief [ADC counts] */
} bno08x_raw_t;

typedef struct
{
    /* Units are uTesla */
    float x; /**< @brief [uTesla] */
    float y; /**< @brief [uTesla] */
    float z; /**< @brief [uTesla] */
} bno08x_magnetic_field_t;

typedef struct
{
    float x;
    float y;
    float z;
} bno08x_accelerometer_t;

typedef struct
{
    float i;        /**< @brief Quaternion component i */
    float j;        /**< @brief Quaternion component j */
    float k;        /**< @brief Quaternion component k */
    float real;     /**< @brief Quaternion component, real */
    float accuracy; /**< @brief Accuracy estimate [radians] */
} bno08x_rotationVector_t;

typedef struct
{
    float i;       /**< @brief Quaternion component i */
    float j;       /**< @brief Quaternion component j */
    float k;       /**< @brief Quaternion component k */
    float real;    /**< @brief Quaternion component real */
    float angVelX; /**< @brief Angular velocity about x [rad/s] */
    float angVelY; /**< @brief Angular velocity about y [rad/s] */
    float angVelZ; /**< @brief Angular velocity about z [rad/s] */
} bno08x_gyroIntegratedRV_t;

typedef enum
{
    UNKNOWN,
    BNO08X_RAW_ACCELEROMETER,
    BNO08X_RAW_MAGNETOMETER,
    BNO08X_RAW_GYROSCOPE,
    BNO08X_ACCELEROMETER,
    BNO08X_MAGNETIC_FIELD,
    BNO08X_ROTATION_VECTOR,
    BNO08X_GYRO_INTEGRATED_RV
} bno08x_sensorType_t;

typedef struct
{
    uint64_t timestamp_uS;
    int64_t delay_uS;
    bno08x_sensorType_t sensorId;
    bno08x_raw_t raw_accelerometer;
    bno08x_raw_t raw_magnetometer;
    bno08x_raw_t raw_gyroscope;
    bno08x_magnetic_field_t magnetic_field;
    bno08x_accelerometer_t accelerometer;
    bno08x_rotationVector_t rotationVector;
    bno08x_rotationVector_t gameRotationVector;
    bno08x_gyroIntegratedRV_t gyroIntegratedRV;
} bno08x_data_t;

typedef void(bno08x_handler_t)(bno08x_data_t *callback);

void bno08x_open(void *hal, bno08x_handler_t *callback);
void bno08x_update();

#endif // BNO08X_H
