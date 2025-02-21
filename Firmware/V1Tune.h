// *********** Initial Default Parameters ********************************** //
// IMU Pre-Filtering
#define IMU_A_ACCEL     0.5f
#define IMU_A_GYRO      0.01f
#define IMU_A_COMP      0.05f
#define IMU_KAL_Q       0.001f
#define IMU_KAL_R       0.03f
#define IMU_Q_BIAS      0.003f
#define IMU_A_XOFFSET   0.036453f
#define IMU_A_YOFFSET   -0.0021066f
#define IMU_A_ZOFFSET   0.133658f
// Motor Speed Control
#define SPEED_KP    2.0f
#define SPEED_KI    12.0f
#define SPEED_KD    0.00f
#define SPEED_ALPHA 0.95f
// Balance Control 
#define BAL_OFFSET  -7.50f  // deg
#define BAL_CUTOFF  35       
#define BAL_KP      0.16
#define BAL_KI      2.0f 
#define BAL_KD      0.002f
// Linear Velocity Control
#define VEL_P       12  
#define VEL_I       8
#define VEL_D       0.03
#define VEL_ALPHA   1.0
// Angular Velocity Control
#define STEER_KP    0.89
#define STEER_KI    0.01
#define STEER_KD    0.000
#define STEER_ALPHA 1.0
// *********** GLOBAL VARIABLES ******************************************** //