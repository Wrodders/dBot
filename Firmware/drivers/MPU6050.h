#ifndef MPU6050_H
#define MPU6050_H


#include "../common/common.h"
#include "i2c.h"
#include <math.h>

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

typedef struct IMUCalib{
    vector_t accel;
    vector_t gyro;
}IMUCalib;

typedef struct MPU6050{
    uint32_t i2c; //STM32 I2C Device handler
    bool initalized;
    uint8_t data; // read data byte
    IMUCalib offset; // Calibration offset
    vector_t accel; //xyz
    vector_t gyro; // xyz
    float pitch, roll;
}MPU6050;


// *** // *** // INTERNAL // *** // **** // 

static void mpu6505Reset(uint32_t i2c){
    // Reset device
    i2cWriteReg(i2c, MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x80); // Set reset bit
    delay(100); // Wait for sensor to reset
    i2cWriteReg(i2c,MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x01); // Auto select clock source
    i2cWriteReg(i2c,MPU6050_ADDR, MPU6050_PWR_MGMT_2, 0x00); // Clear sleep bit
}

static void mpu6050Config(uint32_t i2c){
    // Set Config Registers for MPU6050

    i2cWriteReg(i2c, MPU6050_ADDR, MPU6050_CONFIG , 0x03); // Enable low pass filter 10Hz
    i2cWriteReg(i2c, MPU6050_ADDR, SMPLRT_DIV, 0x04); // Set sample rate to 200Hz

    i2cWriteReg(i2c, MPU6050_ADDR, MPU6050_GYRO_CONFIG, 0x05); // full scale range +/- 500 deg/s
    i2cWriteReg(i2c, MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 0x10); // full scale range +/- 8g
}


static void mpu6050Wake(uint32_t i2c){
    // Wake up device
    i2cWriteReg(i2c, MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00); // Clear sleep bit
    delay(300); // Wait for sensor to stabilize
}

static void gyroCalib(MPU6050 *imu, int samples){
    // calibrates gyro by taking samples and averaging

    imu->offset.gyro.x = 0;
    imu->offset.gyro.y = 0;
    imu->offset.gyro.z = 0;

    uint8_t rawData[6]; // 6 bytes of data
    int16_t gyroRaw[3]; // 3 axes of gyro data

    // Take samples
    for(int i = 0; i < samples; i++){
        // Read raw data
        i2cReadSeq(imu->i2c, MPU6050_ADDR, MPU6050_GYRO_XOUT_H, rawData, 6); 
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
}

static void accelCalib(MPU6050 *imu, int samples){
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
        i2cReadSeq(imu->i2c, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, rawData, 2);
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
        i2cReadSeq(imu->i2c, MPU6050_ADDR, MPU6050_ACCEL_YOUT_H, rawData, 2);
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
        i2cReadSeq(imu->i2c, MPU6050_ADDR, MPU6050_ACCEL_ZOUT_H, rawData, 2);
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
}


// *** // **** // PUBLIC // **** // **** // 


static MPU6050 mpu6050Init(uint32_t perif, uint32_t port, uint32_t scl, uint32_t sda){
    // Initialize MPU6050 Sensor
    MPU6050 imu;
    imu.i2c = i2cInit(perif, port, scl, sda);
    delay(100); 

    mpu6505Reset(imu.i2c);
    mpu6050Wake(imu.i2c);

    // Set Config Registers
    // Read WHO_AM_I register
    uint8_t data = i2cReadReg(imu.i2c, MPU6050_ADDR, MPU6050_WHO_AM_I);
    if (data != MPU6050_ADDR){
        delay(100); // debug
        imu.initalized = false;
        imu.data = data;
        //return imu;
    }
    mpu6050Config(imu.i2c);
    mpu6050Wake(imu.i2c);
    gyroCalib(&imu, 100);
    //accelCalib(imu, 100);
    imu.offset.accel.x = 0.036453f;
    imu.offset.accel.y = -0.0021066f;
    imu.offset.accel.z = 0.133658f;

    imu.initalized = true;

    return imu;
}


static void mpu6050Read(MPU6050 *imu){
    // Read Data from IMU
    // Apply Offsets
    // Data stored in SI Units

    uint8_t rawData[14]; // 14 bytes of data
    // Read raw data
    i2cReadSeq(imu->i2c, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, rawData, 14);

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
}

#endif // MPU6050_H

