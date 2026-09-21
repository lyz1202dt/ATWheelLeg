#ifndef QUATERNION_EKF_H
#define QUATERNION_EKF_H

#include "kalman_filter.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

typedef struct
{
    uint8_t Initialized;
    KalmanFilter_t IMU_QuaternionEKF;
    uint8_t ConvergeFlag;
    uint8_t StableFlag;
    uint64_t ErrorCount;
    uint64_t UpdateCount;

    float q[4];
    float q_prev[4];
    float GyroBias[3];
    float Gyro[3];
    float GyroFiltered[3];
    float GyroPrev[3];
    float GyroAccel[3];
    KalmanFilter_t GyroAccelKF;
    float Accel[3];
    float OrientationCosine[3];

    float accLPFcoef;
    float gyro_norm;
    float accl_norm;
    float AdaptiveGainScale;

    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;

    float Q1;
    float Q2;
    float R;
    float dt;

    mat ChiSquare;
    float ChiSquare_Data[1];
    float ChiSquareTestThreshold;
    float lambda;

    int16_t YawRoundCount;
    float YawAngleLast;

    float P_snapshot[36];
    float K_snapshot[18];
    float H_snapshot[18];
} QEKF_INS_t;

extern QEKF_INS_t QEKF_INS;
extern float chiSquare;
extern float ChiSquareTestThreshold;

void IMU_QuaternionEKF_Init_Instance(QEKF_INS_t *ins,
                                     float process_noise1,
                                     float process_noise2,
                                     float measure_noise,
                                     float lambda,
                                     float dt,
                                     float lpf);
void IMU_QuaternionEKF_Reset_Instance(QEKF_INS_t *ins);
void IMU_QuaternionEKF_Update_Instance(QEKF_INS_t *ins,
                                       float gx, float gy, float gz,
                                       float ax, float ay, float az);

float IMU_QuaternionEKF_GetPitch(const QEKF_INS_t *ins);
float IMU_QuaternionEKF_GetRoll(const QEKF_INS_t *ins);
float IMU_QuaternionEKF_GetYaw(const QEKF_INS_t *ins);
void IMU_QuaternionEKF_GetQuaternion(const QEKF_INS_t *ins, float q[4]);
void IMU_QuaternionEKF_GetAngularVelocity(const QEKF_INS_t *ins, float gyro[3]);
void IMU_QuaternionEKF_GetFilteredAngularVelocity(const QEKF_INS_t *ins, float gyro[3]);
void IMU_QuaternionEKF_GetAngularAcceleration(const QEKF_INS_t *ins, float gyro_accel[3]);

void IMU_QuaternionEKF_Init(float process_noise1,
                            float process_noise2,
                            float measure_noise,
                            float lambda,
                            float dt,
                            float lpf);
void IMU_QuaternionEKF_Update(float gx, float gy, float gz,
                              float ax, float ay, float az);
void IMU_QuaternionEKF_Reset(void);

float Get_Pitch(void);
float Get_Roll(void);
float Get_Yaw(void);
void Get_Quaternion(float q[4]);
void Get_AngularVelocity(float gyro[3]);
void Get_FilteredAngularVelocity(float gyro[3]);
void Get_AngularAcceleration(float gyro_accel[3]);

#ifdef __cplusplus
}
#endif

#endif
