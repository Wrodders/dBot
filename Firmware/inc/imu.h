/**************************************************
 * @brief: IMU Sensor Fusion Library
 * @details: Estimates Pitch and Roll from Accelerometer and Gyroscope Data
 *           Applies 1st order IIR Low Pass Filter to Accelerometer and Gyroscope Data
 *           Provides Kalman and Complementary Filter to Estimate Pitch and Roll
 * @date: 2020/05/31
 * @version: 1.0.0
 * @file: imu.h
 **************************************************/


#ifndef IMU_H
#define IMU_H
#define RAD_TO_DEG 57.295779513082320876798154814105
/* Inertial Measurement State Estimator
Accelerometer and Gyroscope Data Low Pass Filtered. 
Complementary Filter Applied to Fuse Measurements into Angle Estimate
Kalman Filter Applied in 2 Stages: Predict and Update. 
Predict - Apply Kalman Observer Algorithm to latest angle data. 
Roll == X 
Pitch == Y
*/

struct Kalman{
    float Q_angle; // Process Noise
    float Q_bias; // Gyro Bias
    float R_measure; // Measurement Noise
    float angle; // Angle Estimate
    float bias; // Gyro Bias Estimate
    float P[2][2]; // Error Covariance Matrix
}Kalman;

struct IMU{
    struct MPU6050 * sensor;  // IMU Sensor
    const float dt;         // Sample Period [s]
    // Low Pass IIR Filter 
    struct{
        float alpha_accel;          // LFP Alpha 
        float alpha_gyro;           //    
        struct vector_t accel;      // LPF output 
        struct vector_t gyro;      //
    }lpf;
    struct{
        float pitch, roll;  // degreess
    }raw;
    // Complementary Filter
    struct{
        const float a; // Complementary Filter Alpha
        float pitch, roll;
    }comp;
    struct {
        struct Kalman rollK;
        struct Kalman pitchK;
        float pitch, roll;
    }kal;

    float pitchMountOffset; //degrees
    float rollMountOffset;
}IMU; 

static struct IMU imuInit(const float alphaAccel, const float alphaGyro, const float alphaComp, const float dt){
    struct IMU imu = {
        .dt = dt,
        .lpf = {.alpha_accel = alphaAccel, .alpha_gyro = alphaGyro},
        .comp = {.a = alphaComp},
        .kal = {
            .rollK =  {.Q_angle = IMU_KAL_Q, .Q_bias = IMU_Q_BIAS, .R_measure = IMU_KAL_R},
            .pitchK =  {.Q_angle = IMU_KAL_Q, .Q_bias = IMU_Q_BIAS, .R_measure = IMU_KAL_R}
        }
    };
    return imu;
}

static void imuLinkSensor(struct IMU* imu, struct MPU6050* sensor){
    imu->sensor = sensor;
}

static void imuLPF(struct IMU* imu, const struct vector_t* accel, const struct vector_t* gyro){
    // first order IIR filter
    // y[n] = a * y[n-1] + (1-a) * x[n]
    imu->lpf.accel.x = (imu->lpf.alpha_accel * imu->lpf.accel.x) + (1.0f - imu->lpf.alpha_accel) * accel->x;
    imu->lpf.accel.y = (imu->lpf.alpha_accel * imu->lpf.accel.y) + (1.0f - imu->lpf.alpha_accel) * accel->y;
    imu->lpf.accel.z = (imu->lpf.alpha_accel * imu->lpf.accel.z) + (1.0f - imu->lpf.alpha_accel) * accel->z;

    imu->lpf.gyro.x = (imu->lpf.alpha_gyro * imu->lpf.gyro.x) + (1.0f - imu->lpf.alpha_gyro) * gyro->x;
    imu->lpf.gyro.y = (imu->lpf.alpha_gyro * imu->lpf.gyro.y) + (1.0f - imu->lpf.alpha_gyro) * gyro->y;
    imu->lpf.gyro.z = (imu->lpf.alpha_gyro * imu->lpf.gyro.z) + (1.0f - imu->lpf.alpha_gyro) * gyro->z;
}

static void imuCalcRawAngle(struct IMU* imu){
    //@Brief: Computes Euler from raw accelerometer data
    // roll (x-axis rotation)
    float ax2 = imu->lpf.accel.x * imu->lpf.accel.x;
    float ay2 = imu->lpf.accel.y * imu->lpf.accel.y;
    float az2 = imu->lpf.accel.z * imu->lpf.accel.z;
    imu->raw.pitch = atan2f(-imu->lpf.accel.x, imu->lpf.accel.z) * RAD_TO_DEG;
    imu->raw.roll = atan(imu->lpf.accel.y * invSqrt(ax2 + az2)) * RAD_TO_DEG;
}

static void imuCompFilt(struct IMU* imu){
    //@Brief: Complementary Filter Converts imu data into Euler Angles
    imuCalcRawAngle(imu);
    // Pitch
    imu->comp.pitch = imu->comp.a * (imu->comp.pitch - imu->lpf.gyro.z * imu->dt) 
                    + (1-imu->comp.a) * imu->raw.pitch;
    // Roll
    imu->comp.roll = imu->comp.a * (imu->comp.roll - imu->lpf.gyro.x * imu->dt) 
                    + (1-imu->comp.a) * imu->raw.roll;
}

static float imuKalPredict(struct Kalman* kal, const float newAngle, const float newRate, float const dt) {
    // Predicts an angle via kalman filter algorithm from angle[n] and dy/dt angle[n]
    float rate = newRate - kal->bias;
    kal->angle += dt * rate;

    kal->P[0][0] += dt * (dt * kal->P[1][1] - kal->P[0][1] - kal->P[1][0] + kal->Q_angle);
    kal->P[0][1] -= dt * kal->P[1][1];
    kal->P[1][0] -= dt * kal->P[1][1];
    kal->P[1][1] += kal->Q_bias * dt; // 

    float S = kal->P[0][0] + kal->R_measure;
    float K[2];
    K[0] = kal->P[0][0] / S;
    K[1] = kal->P[1][0] / S;

    float y = newAngle - kal->angle;
    kal->angle += K[0] * y;
    kal->bias += K[1] * y;

    float P00_temp = kal->P[0][0];
    float P01_temp = kal->P[0][1];

    kal->P[0][0] -= K[0] * P00_temp;
    kal->P[0][1] -= K[0] * P01_temp;
    kal->P[1][0] -= K[1] * P00_temp;
    kal->P[1][1] -= K[1] * P01_temp;

    return kal->angle;
}

static void imuKalUpdate(struct IMU* imu){
    // Update Kalman Filter with latest data
    float xRate = imu->lpf.gyro.x;
    float yRate = imu->lpf.gyro.y;
    float ax = imu->lpf.accel.x;
    float ay = imu->lpf.accel.y;
    float az = imu->lpf.accel.z;
    float ax2 = ax * ax;
    float az2 = az * az;

    float roll_estimate; // Compute Roll estimate from lpf accelerometer
    float rollsqrt = sqrt(ax2 + az2);
    if(rollsqrt != 0.0f){roll_estimate = atan(ay / rollsqrt) * RAD_TO_DEG;} // Fix -180=180
    else{roll_estimate = 0.0f;} 
    if (fabs(imu->kal.pitch) > 90){xRate *= -1;} // Invert Rate if Pitch is > 90   
    imu->kal.roll= imuKalPredict(&imu->kal.rollK, roll_estimate, xRate, imu->dt);

    // Estimate Pitch From Accelerometer
    float pitch_estimate = atan2(-ax, az ) * RAD_TO_DEG;
    if ((pitch_estimate < -90 && imu->kal.pitch > 90) || (pitch_estimate > 90 && imu->kal.pitch < -90)){
        imu->kal.pitchK.angle = pitch_estimate; // Reset the Angle when the IMU has moved 180 degrees
        imu->kal.pitch = pitch_estimate; 
    }
    else{
        imu->kal.pitch = imuKalPredict(&imu->kal.pitchK, pitch_estimate, yRate, imu->dt);
    }
}


static void imuUpdate(struct IMU* imu){
    mpu6050Read(imu->sensor); // Read Sensor need to move this to Interrupt Based
    
    imuLPF(imu, &imu->sensor->accel, &imu->sensor->gyro); // Apply Low pass filter
    imuKalUpdate(imu); // Estimate Angle with Kalman Observer
}



#endif // IMU_H