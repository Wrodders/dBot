#ifndef MPU6060_H
#define MPU6050_H


#include "../common/common.h"
#include "../drivers/i2c.h"
#include <math.h>



typedef struct vector_t{
    float x,y,z;
}vector_t;


// MPU6050 Config Registers
#define MPU6050_ADDR 0x68 // MPU6050 I2C Address
#define MPU6050_WHO_AM_I 0x75 // Returns address of device (0x68)
#define MPU6050_PWR_MGMT_1 0x6B // Power management register
#define MPU6050_PWR_MGMT_2 0x6C // Power management register
#define MPU6050_CONFIG 0x1A // Configuration register
#define MPU6050_GYRO_CONFIG 0x1B // Gyro config register
#define MPU6050_ACCEL_CONFIG 0x1C // Accel config register
#define SMPLRT_DIV 0x19 // Sample rate divider register

// MPU6050 Data Registers
#define MPU6050_ACCEL_XOUT_H 0x3B // Accel X axis high byte
#define MPU6050_ACCEL_YOUT_H 0x3D // Accel Y axis high byte
#define MPU6050_ACCEL_ZOUT_H 0x3F // Accel Z axis high byte
#define MPU6050_TEMP_OUT_H 0x41 // Temperature high byte
#define MPU6050_GYRO_XOUT_H 0x43 // Gyro X axis high byte

// MPU6050 Functions

typedef struct Calib_t{
    int valid; // must read 77 to be valid
    vector_t accel;
    vector_t gyro;
}Calib_t;

typedef struct MPU6050_t{
    uint8_t data;
    bool initalized;
    vector_t accel;
    vector_t gyro;
    Calib_t offset; // Calibration offset
    float roll, pitch; // estimation results
    uint32_t i2c;
}MPU6050_t;



static void resetMPU6050(uint32_t i2c){
    // Reset device
    _i2c_write_reg(i2c, MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x80); // Set reset bit
    delay(100); // Wait for sensor to reset
    _i2c_write_reg(i2c,MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x01); // Auto select clock source
    _i2c_write_reg(i2c,MPU6050_ADDR, MPU6050_PWR_MGMT_2, 0x00); // Clear sleep bit
    return;
}

static void configMPU6050(uint32_t i2c){
    // Set Config Registers for MPU6050

    _i2c_write_reg(i2c, MPU6050_ADDR, MPU6050_CONFIG , 0x03); // Enable low pass filter 10Hz
    _i2c_write_reg(i2c, MPU6050_ADDR, SMPLRT_DIV, 0x04); // Set sample rate to 200Hz

    _i2c_write_reg(i2c, MPU6050_ADDR, MPU6050_GYRO_CONFIG, 0x05); // full scale range +/- 500 deg/s
    _i2c_write_reg(i2c, MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 0x10); // full scale range +/- 8g
    return;
}


static void wakeMPU6050(uint32_t i2c){
    // Wake up device
    _i2c_write_reg(i2c, MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00); // Clear sleep bit
    delay(300); // Wait for sensor to stabilize
    return;
}

static void calibGyro(MPU6050_t *imu, int samples){
    // calibrates gyro by taking samples and averaging

    imu->offset.gyro.x = 0;
    imu->offset.gyro.y = 0;
    imu->offset.gyro.z = 0;

    uint8_t rawData[6]; // 6 bytes of data
    int16_t gyroRaw[3]; // 3 axes of gyro data

    // Take samples
    for(int i = 0; i < samples; i++){
        // Read raw data
        i2c_read_seq(imu->i2c, MPU6050_ADDR, MPU6050_GYRO_XOUT_H, rawData, 6); 
        // convert to 16 bit signed integers
        gyroRaw[0] = (int16_t)((rawData[0] << 8) | rawData[1]); // Gyro X
        gyroRaw[1] = (int16_t)((rawData[2] << 8) | rawData[3]); // Gyro Y
        gyroRaw[2] = (int16_t)((rawData[4] << 8) | rawData[5]); // Gyro Z
        // Add to offset
        imu->offset.gyro.x += (float)gyroRaw[0] / 65.5f;
        imu->offset.gyro.y += (float)gyroRaw[1] / 65.5f;
        imu->offset.gyro.z += (float)gyroRaw[2] / 65.5f;
        delay(3);
    }

    // Average offset
    imu->offset.gyro.x /= samples;
    imu->offset.gyro.y /= samples;
    imu->offset.gyro.z /= samples;
    return;
}

static void calibAccel(MPU6050_t *imu, int samples){
    // calibrates accel by taking samples and averaging
    // should callc offests to set gry to close to 1g on each axis
    imu->offset.accel.x = 0;
    imu->offset.accel.y = 0;
    imu->offset.accel.z = 0;

    uint8_t rawData[2]; // 2 bytes of data
    int16_t accelRaw[3]; // 3 axes of accel data

    // Take samples
    //printf("Calibrating AccelX: \n");
    delay(1000);
    for(int i = 0; i< samples; i++){
        // Read raw data
        i2c_read_seq(imu->i2c, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, rawData, 2);
        // convert to 16 bit signed integers
        accelRaw[0] = (int16_t)((rawData[0] << 8) | rawData[1]); // Accel X
        // Add to offset
        imu->offset.accel.x += (float)accelRaw[0] / 4096.0f;
        delay(3);
    }
    //printf("Calibrating AccelY: \n");
    delay(1000);

    for(int i = 0; i < samples ; i++){
        // Read raw data
        i2c_read_seq(imu->i2c, MPU6050_ADDR, MPU6050_ACCEL_YOUT_H, rawData, 2);
        // convert to 16 bit signed integers
        accelRaw[1] = (int16_t)((rawData[0] << 8) | rawData[1]); // Accel Y
        // Add to offset
        imu->offset.accel.y += (float)accelRaw[1] / 4096.0f;
        delay(3);
    }
    //printf("Calibrating AccelZ: \n");
    delay(1000);
    for(int i = 0; i < samples; i++){
        // Read raw data
        i2c_read_seq(imu->i2c, MPU6050_ADDR, MPU6050_ACCEL_ZOUT_H, rawData, 2);
        // convert to 16 bit signed integers
        accelRaw[2] = (int16_t)((rawData[0] << 8) | rawData[1]); // Accel Z
        // Add to offset
        imu->offset.accel.z += (float)accelRaw[2] / 4096.0f;
        delay(3);
    }

    //Callibration complete

    // Average offset
    imu->offset.accel.x /= samples;
    imu->offset.accel.y /= samples;
    imu->offset.accel.z /= samples;
    return;

}

static MPU6050_t initMPU6050(uint32_t sda, uint32_t scl, uint32_t port){
    // Initialize MPU6050 Sensor
    MPU6050_t imu;
    imu.i2c = i2c_setup(I2C1, port, sda, scl);
    delay(100); 

    // Set Config Registers
    // Read WHO_AM_I register
    uint8_t data = _i2c_read_reg(imu.i2c, MPU6050_ADDR, MPU6050_WHO_AM_I);
    if (data != MPU6050_ADDR){
        delay(100); // debug
        imu.initalized = false;
        imu.data = data;
        //return imu;
    }
    configMPU6050(imu.i2c);
    wakeMPU6050(imu.i2c);
    calibGyro(&imu, 100);
    //calibAccel(imu, 100);
    imu.offset.accel.x = 0.036453f;
    imu.offset.accel.y = -0.0021066f;
    imu.offset.accel.z = 0.133658f;

    return imu;
}


static void readMPU6050(MPU6050_t *imu){
    // Read Data from IMU
    // Apply Offsets
    // Data stored in SI Units

    uint8_t rawData[14]; // 14 bytes of data
    // Read raw data
    i2c_read_seq(imu->i2c, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, rawData, 14);

    int16_t accelRaw[3]; // 3 axes of accel data
    int16_t gyroRaw[3]; // 3 axes of gyro data

    // Convert to 16 bit signed integers
    accelRaw[0] = (int16_t)((rawData[0] << 8) | rawData[1]); // Accel X
    accelRaw[1] = (int16_t)((rawData[2] << 8) | rawData[3]); // Accel Y
    accelRaw[2] = (int16_t)((rawData[4] << 8) | rawData[5]); // Accel Z
    gyroRaw[0] = (int16_t)((rawData[8] << 8) | rawData[9]); // Gyro X
    gyroRaw[1] = (int16_t)((rawData[10] << 8) | rawData[11]); // Gyro Y
    gyroRaw[2] = (int16_t)((rawData[12] << 8) | rawData[13]); // Gyro Z

    // Convert to SI units
    imu->accel.x = (float)accelRaw[0] / 4096.0f;
    imu->accel.y = (float)accelRaw[1] / 4096.0f;
    imu->accel.z = (float)accelRaw[2] / 4096.0f;
    imu->gyro.x = (float)gyroRaw[0] / 65.5f;
    imu->gyro.y = (float)gyroRaw[1] / 65.5f;
    imu->gyro.z = (float)gyroRaw[2] / 65.5f; 

    // Apply calibration offsets
    imu->accel.x -= imu->offset.accel.x;
    imu->accel.y -= imu->offset.accel.y;
    imu->accel.z -= imu->offset.accel.z;

    imu->gyro.x -= imu->offset.gyro.x;
    imu->gyro.y -= imu->offset.gyro.y;
    imu->gyro.z -= imu->offset.gyro.z;
    return;
}

static void getRawAngle(vector_t *accel, float *roll, float *pitch){

    // roll (x-axis rotation)
    float ax2 = accel->x * accel->x;
    float ay2 = accel->y * accel->y;
    float az2 = accel->z * accel->z;


    *roll = atan(accel->y * invSqrt(ax2 + az2));
    // pitch (y-axis rotation)
    *pitch = atan(accel->x * invSqrt(ay2 + az2));
    return;
}


// ********* Madgwick Filter Functions *****************************************************// 
typedef struct MadgwickFilter {
  float    beta;
  float    q0;
  float    q1;
  float    q2;
  float    q3;
  float    freq;
  float    inv_freq;
  uint32_t counter;
}MadgwickFilter;


// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

static float invSqrt(float x) {
    union {
    float    f;
    uint32_t i;
    } conv;

    float       x2;
    const float threehalfs = 1.5F;

    x2     = x * 0.5F;
    conv.f = x;
    conv.i = 0x5f3759df - (conv.i >> 1); // ????
    conv.f = conv.f * (threehalfs - (x2 * conv.f * conv.f));
    return conv.f;
}

static MadgwickFilter initMadgwickfilter(void) {
    MadgwickFilter filter;
    configMadqwickFilter(&filter, 100.0f, 0.1f);
    resetMadgwickFilter(&filter);
    return filter;
}

static bool configMadqwickFilter(MadgwickFilter *filter, float freq, float beta){
    if (!filter) {
    return false;
  }
  filter->beta     = beta;
  filter->freq     = freq;
  filter->inv_freq = 1.0f / freq;
  return true;
}


static bool resetMadgwickFilter(MadgwickFilter *filter){
     if (!filter) {
    return false;
  }
  filter->q0      = 1.0f;
  filter->q1      = 0.0f;
  filter->q2      = 0.0f;
  filter->q3      = 0.0f;
  filter->counter = 0;
  return true;
}

static bool updateMadgwickFilter(MadgwickFilter *filter, float gx, float gy, float gz, float ax, float ay, float az){
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // No need to check filter pointer -- it's checked by _update()

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-filter->q1 * gx - filter->q2 * gy - filter->q3 * gz);
    qDot2 = 0.5f * (filter->q0 * gx + filter->q2 * gz - filter->q3 * gy);
    qDot3 = 0.5f * (filter->q0 * gy - filter->q1 * gz + filter->q3 * gx);
    qDot4 = 0.5f * (filter->q0 * gz + filter->q1 * gy - filter->q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax       *= recipNorm;
    ay       *= recipNorm;
    az       *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * filter->q0;
    _2q1 = 2.0f * filter->q1;
    _2q2 = 2.0f * filter->q2;
    _2q3 = 2.0f * filter->q3;
    _4q0 = 4.0f * filter->q0;
    _4q1 = 4.0f * filter->q1;
    _4q2 = 4.0f * filter->q2;
    _8q1 = 8.0f * filter->q1;
    _8q2 = 8.0f * filter->q2;
    q0q0 = filter->q0 * filter->q0;
    q1q1 = filter->q1 * filter->q1;
    q2q2 = filter->q2 * filter->q2;
    q3q3 = filter->q3 * filter->q3;

    // Gradient decent algorithm corrective step
    s0        = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1        = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * filter->q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2        = 4.0f * q0q0 * filter->q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3        = 4.0f * q1q1 * filter->q3 - _2q1 * ax + 4.0f * q2q2 * filter->q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);             // normalise step magnitude
    s0       *= recipNorm;
    s1       *= recipNorm;
    s2       *= recipNorm;
    s3       *= recipNorm;

    // Apply feedback step
    qDot1 -= filter->beta * s0;
    qDot2 -= filter->beta * s1;
    qDot3 -= filter->beta * s2;
    qDot4 -= filter->beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    filter->q0 += qDot1 * filter->inv_freq;
    filter->q1 += qDot2 * filter->inv_freq;
    filter->q2 += qDot3 * filter->inv_freq;
    filter->q3 += qDot4 * filter->inv_freq;

    // Normalise quaternion
    recipNorm   = invSqrt(filter->q0 * filter->q0 + filter->q1 * filter->q1 + filter->q2 * filter->q2 + filter->q3 * filter->q3);
    filter->q0 *= recipNorm;
    filter->q1 *= recipNorm;
    filter->q2 *= recipNorm;
    filter->q3 *= recipNorm;

    filter->counter++;
    return true;

}


static bool computeEulerAngles(MadgwickFilter *filter, float *roll, float *pitch, float *yaw){
    if (!filter) {return false;}
    // Calc Angles
    *roll = asinf(-2.0f * (filter->q1 * filter->q3 - filter->q0 * filter->q2));
    *pitch = atan2f(filter->q0 * filter->q1 + filter->q2 * filter->q3, 0.5f - filter->q1 * filter->q1 - filter->q2 * filter->q2);
    *yaw = atan2f(filter->q1 * filter->q2 + filter->q0 * filter->q3, 0.5f - filter->q2 * filter->q2 - filter->q3 * filter->q3);
    
    return true;
}

#endif

