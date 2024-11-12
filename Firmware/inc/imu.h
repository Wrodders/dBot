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

typedef struct Kalman{
    float Q_angle;
    float Q_bias;
    float R_measure;
    float angle;
    float bias;
    float P[2][2];
}Kalman;

typedef struct IMU{
    const float dt;         // Sample Period [s]
    // Low Pass IIR Filter 
    struct{
        float aAccel;       // LFP Alpha 
        float aGyro;        //    
        vector_t accel;     // LPF output 
        vector_t gyro;      //
    }lpf;
    struct{
        float pitch, roll;
    }raw;
    // Complementary Filter
    struct{
        const float a;
        float pitch, roll;
    }comp;
    struct {
        Kalman X;
        Kalman Y;
        float pitch, roll;
    }kal;
}IMU; 

static IMU imuInit(const float alphaAccel, const float alphaGyro, const float alphaComp, const float dt){
    IMU imu = {
        .dt = dt,
        .lpf = {.aAccel = alphaAccel, .aGyro = alphaGyro},
        .comp = {.a = alphaComp},
        .kal = {
            .X =  {.Q_angle = 0.001f, .Q_bias = 0.003f, .R_measure = 0.03f},
            .Y =  {.Q_angle = 0.001f, .Q_bias = 0.003f, .R_measure = 0.03f}
        }
    };
    return imu;
}

static void imuLPF(IMU* imu, const vector_t* accel, const vector_t* gyro){
    // first order IIR filter
    // y[n] = a * y[n-1] + (1-a) * x[n]
    imu->lpf.accel.x = (imu->lpf.aAccel * imu->lpf.accel.x) + (1.0f - imu->lpf.aAccel) * accel->x;
    imu->lpf.accel.y = (imu->lpf.aAccel * imu->lpf.accel.y) + (1.0f - imu->lpf.aAccel) * accel->y;
    imu->lpf.accel.z = (imu->lpf.aAccel * imu->lpf.accel.z) + (1.0f - imu->lpf.aAccel) * accel->z;

    imu->lpf.gyro.x = (imu->lpf.aGyro * imu->lpf.gyro.x) + (1.0f - imu->lpf.aGyro) * gyro->x;
    imu->lpf.gyro.y = (imu->lpf.aGyro * imu->lpf.gyro.y) + (1.0f - imu->lpf.aGyro) * gyro->y;
    imu->lpf.gyro.z = (imu->lpf.aGyro * imu->lpf.gyro.z) + (1.0f - imu->lpf.aGyro) * gyro->z;
}

static void imuCalcRawAngle(IMU* imu){
    //@Brief: Computes Euler from raw accelerometer data
    // roll (x-axis rotation)
    float ax2 = imu->lpf.accel.x * imu->lpf.accel.x;
    float ay2 = imu->lpf.accel.y * imu->lpf.accel.y;
    float az2 = imu->lpf.accel.z * imu->lpf.accel.z;
    imu->raw.pitch = atan2f(-imu->lpf.accel.x, imu->lpf.accel.z) * RAD_TO_DEG;
    imu->raw.roll = atan(imu->lpf.accel.y * invSqrt(ax2 + az2)) * RAD_TO_DEG;
}

static void imuCompFilt(IMU* imu){
    //@Brief: Complementary Filter Converts imu data into Euler Angles
    imuCalcRawAngle(imu);
    // Pitch
    imu->comp.pitch = imu->comp.a * (imu->comp.pitch - imu->lpf.gyro.z * imu->dt) 
                    + (1-imu->comp.a) * imu->raw.pitch;
    // Roll
    imu->comp.roll = imu->comp.a * (imu->comp.roll - imu->lpf.gyro.x * imu->dt) 
                    + (1-imu->comp.a) * imu->raw.roll;

}

static float imuKalPredict(Kalman* kal, const float newAngle, const float newRate, float const dt) {
    // Predicts an angle via kalman filter algorithm from angle[n] and dy/dt angle[n]
    //
    float rate = newRate - kal->bias;
    kal->angle += dt * rate;

    kal->P[0][0] += dt * (dt * kal->P[1][1] - kal->P[0][1] - kal->P[1][0] + kal->Q_angle);
    kal->P[0][1] -= dt * kal->P[1][1];
    kal->P[1][0] -= dt * kal->P[1][1];
    kal->P[1][1] += kal->Q_bias * dt;

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

static void imuKalUpdate(IMU* imu){
    float xRate = imu->lpf.gyro.x;
    float yRate = imu->lpf.gyro.y;
    float ax = imu->lpf.accel.x;
    float ay = imu->lpf.accel.y;
    float az = imu->lpf.accel.z;
    float ax2 = ax * ax;
    float az2 = az * az;
    // Kalman angle solver
    // Compute Roll estimate from lpf accelerometer
    float roll;
    float rollsqrt = sqrt(ax2 + az2);
    if(rollsqrt != 0.0f){roll = atan(ay / rollsqrt) * RAD_TO_DEG;} // Fix -180=180
    else{roll = 0.0f;}

  
    if (fabs(imu->kal.pitch) > 90){
        xRate *= -1;
    }   
    imu->kal.roll= imuKalPredict(&imu->kal.X, roll, xRate, imu->dt);

    // Estimate Pitch From Accelerometer
    float pitch = atan2(-ax, az ) * RAD_TO_DEG;
    if ((pitch < -90 && imu->kal.pitch > 90) || (pitch > 90 && imu->kal.pitch < -90)){
        imu->kal.Y.angle = pitch;
        imu->kal.pitch = pitch;
    }
    else{
        imu->kal.pitch = imuKalPredict(&imu->kal.Y, pitch, yRate, imu->dt);
    }
}


#endif // IMU_H