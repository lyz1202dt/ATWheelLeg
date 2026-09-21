#include "QuaternionEKF.h"

#include <math.h>
#include <string.h>

#define DEG_PER_RAD 57.29577951308232f
#define GYRO_ACCEL_KF_Q 500.0f
#define GYRO_ACCEL_KF_R 3000.0f

QEKF_INS_t QEKF_INS = {0};
float chiSquare = 0.0f;
float ChiSquareTestThreshold = 1e-8f;

static const float IMU_QuaternionEKF_F[36] = {
    1, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1,
};

static const float IMU_QuaternionEKF_P_Const[36] = {
    100000, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f,
    0.1f, 100000, 0.1f, 0.1f, 0.1f, 0.1f,
    0.1f, 0.1f, 100000, 0.1f, 0.1f, 0.1f,
    0.1f, 0.1f, 0.1f, 100000, 0.1f, 0.1f,
    0.1f, 0.1f, 0.1f, 0.1f, 100, 0.1f,
    0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 100,
};

static const float IMU_GyroAccelKF_F[9] = {
    1.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 1.0f,
};

static const float IMU_GyroAccelKF_H[9] = {
    1.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 1.0f,
};

static const float IMU_GyroAccelKF_P_Init[9] = {
    100.0f, 0.0f, 0.0f,
    0.0f, 100.0f, 0.0f,
    0.0f, 0.0f, 100.0f,
};

static QEKF_INS_t *ownerFromFilter(KalmanFilter_t *kf);
static float invSqrt(float x);
static float clampf(float value, float low, float high);
static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_CalculateFilteredGyro(QEKF_INS_t *ins);
static void resetFilterState(QEKF_INS_t *ins, int filters_ready);
static void configureFilters(QEKF_INS_t *ins);

void IMU_QuaternionEKF_Init_Instance(QEKF_INS_t *ins,
                                     float process_noise1,
                                     float process_noise2,
                                     float measure_noise,
                                     float lambda,
                                     float dt,
                                     float lpf)
{
    if (ins == NULL) {
        return;
    }

    const int filters_ready = (ins->Initialized != 0U &&
                               ins->IMU_QuaternionEKF.xhat_data != NULL &&
                               ins->GyroAccelKF.xhat_data != NULL);

    resetFilterState(ins, filters_ready);

    ins->Initialized = 1U;
    ins->Q1 = process_noise1;
    ins->Q2 = process_noise2;
    ins->R = measure_noise;
    ins->ChiSquareTestThreshold = ChiSquareTestThreshold;
    ins->dt = (dt > 0.0f) ? dt : 0.001f;
    ins->accLPFcoef = lpf;
    ins->lambda = (lambda > 0.0f && lambda <= 1.0f) ? lambda : 1.0f;

    ins->IMU_QuaternionEKF.xhat_data[0] = 1.0f;
    ins->q[0] = 1.0f;
    ins->q_prev[0] = 1.0f;

    configureFilters(ins);
}

void IMU_QuaternionEKF_Reset_Instance(QEKF_INS_t *ins)
{
    if (ins == NULL) {
        return;
    }

    const float q1 = ins->Q1;
    const float q2 = ins->Q2;
    const float r = ins->R;
    const float lambda = ins->lambda;
    const float dt = ins->dt;
    const float lpf = ins->accLPFcoef;
    const int filters_ready = (ins->Initialized != 0U &&
                               ins->IMU_QuaternionEKF.xhat_data != NULL &&
                               ins->GyroAccelKF.xhat_data != NULL);

    resetFilterState(ins, filters_ready);

    ins->Initialized = 1U;
    ins->Q1 = q1;
    ins->Q2 = q2;
    ins->R = r;
    ins->lambda = lambda;
    ins->dt = dt;
    ins->accLPFcoef = lpf;
    ins->ChiSquareTestThreshold = ChiSquareTestThreshold;

    ins->IMU_QuaternionEKF.xhat_data[0] = 1.0f;
    ins->q[0] = 1.0f;
    ins->q_prev[0] = 1.0f;

    configureFilters(ins);
}

void IMU_QuaternionEKF_Update_Instance(QEKF_INS_t *ins,
                                       float gx, float gy, float gz,
                                       float ax, float ay, float az)
{
    volatile float halfgxdt;
    volatile float halfgydt;
    volatile float halfgzdt;
    volatile float accelInvNorm;

    if (ins == NULL) {
        return;
    }

    if (ins->Initialized == 0U) {
        IMU_QuaternionEKF_Init_Instance(ins, 10.0f, 0.001f, 10000000.0f,
                                        1.0f, 0.001f, 0.0f);
    }

    ins->Gyro[0] = gx - ins->GyroBias[0];
    ins->Gyro[1] = gy - ins->GyroBias[1];
    ins->Gyro[2] = gz - ins->GyroBias[2];

    halfgxdt = 0.5f * ins->Gyro[0] * ins->dt;
    halfgydt = 0.5f * ins->Gyro[1] * ins->dt;
    halfgzdt = 0.5f * ins->Gyro[2] * ins->dt;

    memcpy(ins->IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));

    ins->IMU_QuaternionEKF.F_data[1] = -halfgxdt;
    ins->IMU_QuaternionEKF.F_data[2] = -halfgydt;
    ins->IMU_QuaternionEKF.F_data[3] = -halfgzdt;

    ins->IMU_QuaternionEKF.F_data[6] = halfgxdt;
    ins->IMU_QuaternionEKF.F_data[8] = halfgzdt;
    ins->IMU_QuaternionEKF.F_data[9] = -halfgydt;

    ins->IMU_QuaternionEKF.F_data[12] = halfgydt;
    ins->IMU_QuaternionEKF.F_data[13] = -halfgzdt;
    ins->IMU_QuaternionEKF.F_data[15] = halfgxdt;

    ins->IMU_QuaternionEKF.F_data[18] = halfgzdt;
    ins->IMU_QuaternionEKF.F_data[19] = halfgydt;
    ins->IMU_QuaternionEKF.F_data[20] = -halfgxdt;

    if (ins->UpdateCount == 0U || ins->accLPFcoef <= 0.0f) {
        ins->Accel[0] = ax;
        ins->Accel[1] = ay;
        ins->Accel[2] = az;
    } else {
        const float keep = ins->accLPFcoef / (ins->dt + ins->accLPFcoef);
        const float input = ins->dt / (ins->dt + ins->accLPFcoef);
        ins->Accel[0] = ins->Accel[0] * keep + ax * input;
        ins->Accel[1] = ins->Accel[1] * keep + ay * input;
        ins->Accel[2] = ins->Accel[2] * keep + az * input;
    }

    ins->accl_norm = sqrtf(ins->Accel[0] * ins->Accel[0] +
                           ins->Accel[1] * ins->Accel[1] +
                           ins->Accel[2] * ins->Accel[2]);
    if (ins->accl_norm < 1.0e-6f) {
        return;
    }
    accelInvNorm = 1.0f / ins->accl_norm;

    ins->IMU_QuaternionEKF.MeasuredVector[0] = ins->Accel[0] * accelInvNorm;
    ins->IMU_QuaternionEKF.MeasuredVector[1] = ins->Accel[1] * accelInvNorm;
    ins->IMU_QuaternionEKF.MeasuredVector[2] = ins->Accel[2] * accelInvNorm;

    ins->gyro_norm = sqrtf(ins->Gyro[0] * ins->Gyro[0] +
                           ins->Gyro[1] * ins->Gyro[1] +
                           ins->Gyro[2] * ins->Gyro[2]);

    ins->StableFlag = (ins->gyro_norm < 0.3f &&
                       ins->accl_norm > 9.3f &&
                       ins->accl_norm < 10.3f)
                          ? 1U
                          : 0U;

    ins->IMU_QuaternionEKF.Q_data[0] = ins->Q1 * ins->dt;
    ins->IMU_QuaternionEKF.Q_data[7] = ins->Q1 * ins->dt;
    ins->IMU_QuaternionEKF.Q_data[14] = ins->Q1 * ins->dt;
    ins->IMU_QuaternionEKF.Q_data[21] = ins->Q1 * ins->dt;
    ins->IMU_QuaternionEKF.Q_data[28] = ins->Q2 * ins->dt;
    ins->IMU_QuaternionEKF.Q_data[35] = ins->Q2 * ins->dt;
    ins->IMU_QuaternionEKF.R_data[0] = ins->R;
    ins->IMU_QuaternionEKF.R_data[4] = ins->R;
    ins->IMU_QuaternionEKF.R_data[8] = ins->R;

    Kalman_Filter_Update(&ins->IMU_QuaternionEKF);

    ins->q[0] = ins->IMU_QuaternionEKF.FilteredValue[0];
    ins->q[1] = ins->IMU_QuaternionEKF.FilteredValue[1];
    ins->q[2] = ins->IMU_QuaternionEKF.FilteredValue[2];
    ins->q[3] = ins->IMU_QuaternionEKF.FilteredValue[3];

    ins->Roll = DEG_PER_RAD * atan2f(ins->q[0] * ins->q[1] + ins->q[2] * ins->q[3],
                                     0.5f - ins->q[1] * ins->q[1] - ins->q[2] * ins->q[2]);
    ins->Pitch = DEG_PER_RAD * asinf(clampf(-2.0f * (ins->q[1] * ins->q[3] -
                                                     ins->q[0] * ins->q[2]),
                                           -1.0f, 1.0f));
    ins->Yaw = DEG_PER_RAD * atan2f(ins->q[1] * ins->q[2] + ins->q[0] * ins->q[3],
                                    0.5f - ins->q[2] * ins->q[2] - ins->q[3] * ins->q[3]);

    ins->GyroBias[0] = ins->IMU_QuaternionEKF.FilteredValue[4];
    ins->GyroBias[1] = ins->IMU_QuaternionEKF.FilteredValue[5];
    ins->GyroBias[2] = 0.0f;

    IMU_QuaternionEKF_CalculateFilteredGyro(ins);

    if (ins->UpdateCount > 0U) {
        ins->GyroAccelKF.MeasuredVector[0] = (ins->GyroFiltered[0] - ins->GyroPrev[0]) / ins->dt;
        ins->GyroAccelKF.MeasuredVector[1] = (ins->GyroFiltered[1] - ins->GyroPrev[1]) / ins->dt;
        ins->GyroAccelKF.MeasuredVector[2] = (ins->GyroFiltered[2] - ins->GyroPrev[2]) / ins->dt;
    } else {
        ins->GyroAccelKF.MeasuredVector[0] = 0.0f;
        ins->GyroAccelKF.MeasuredVector[1] = 0.0f;
        ins->GyroAccelKF.MeasuredVector[2] = 0.0f;
    }

    Kalman_Filter_Update(&ins->GyroAccelKF);
    ins->GyroAccel[0] = ins->GyroAccelKF.FilteredValue[0];
    ins->GyroAccel[1] = ins->GyroAccelKF.FilteredValue[1];
    ins->GyroAccel[2] = ins->GyroAccelKF.FilteredValue[2];

    ins->GyroPrev[0] = ins->GyroFiltered[0];
    ins->GyroPrev[1] = ins->GyroFiltered[1];
    ins->GyroPrev[2] = ins->GyroFiltered[2];

    if (ins->Yaw - ins->YawAngleLast > 180.0f) {
        ins->YawRoundCount--;
    } else if (ins->Yaw - ins->YawAngleLast < -180.0f) {
        ins->YawRoundCount++;
    }
    ins->YawTotalAngle = 360.0f * (float)ins->YawRoundCount + ins->Yaw;
    ins->YawAngleLast = ins->Yaw;

    ins->q_prev[0] = ins->q[0];
    ins->q_prev[1] = ins->q[1];
    ins->q_prev[2] = ins->q[2];
    ins->q_prev[3] = ins->q[3];

    ins->UpdateCount++;
}

float IMU_QuaternionEKF_GetPitch(const QEKF_INS_t *ins)
{
    return ins != NULL ? ins->Pitch : 0.0f;
}

float IMU_QuaternionEKF_GetRoll(const QEKF_INS_t *ins)
{
    return ins != NULL ? ins->Roll : 0.0f;
}

float IMU_QuaternionEKF_GetYaw(const QEKF_INS_t *ins)
{
    return ins != NULL ? ins->Yaw : 0.0f;
}

void IMU_QuaternionEKF_GetQuaternion(const QEKF_INS_t *ins, float q[4])
{
    if (ins == NULL || q == NULL) {
        return;
    }

    q[0] = ins->q[0];
    q[1] = ins->q[1];
    q[2] = ins->q[2];
    q[3] = ins->q[3];
}

void IMU_QuaternionEKF_GetAngularVelocity(const QEKF_INS_t *ins, float gyro[3])
{
    if (ins == NULL || gyro == NULL) {
        return;
    }

    gyro[0] = ins->GyroFiltered[0];
    gyro[1] = ins->GyroFiltered[1];
    gyro[2] = ins->GyroFiltered[2];
}

void IMU_QuaternionEKF_GetFilteredAngularVelocity(const QEKF_INS_t *ins, float gyro[3])
{
    IMU_QuaternionEKF_GetAngularVelocity(ins, gyro);
}

void IMU_QuaternionEKF_GetAngularAcceleration(const QEKF_INS_t *ins, float gyro_accel[3])
{
    if (ins == NULL || gyro_accel == NULL) {
        return;
    }

    gyro_accel[0] = ins->GyroAccel[0];
    gyro_accel[1] = ins->GyroAccel[1];
    gyro_accel[2] = ins->GyroAccel[2];
}

void IMU_QuaternionEKF_Init(float process_noise1,
                            float process_noise2,
                            float measure_noise,
                            float lambda,
                            float dt,
                            float lpf)
{
    IMU_QuaternionEKF_Init_Instance(&QEKF_INS, process_noise1, process_noise2,
                                    measure_noise, lambda, dt, lpf);
}

void IMU_QuaternionEKF_Update(float gx, float gy, float gz,
                              float ax, float ay, float az)
{
    IMU_QuaternionEKF_Update_Instance(&QEKF_INS, gx, gy, gz, ax, ay, az);
}

void IMU_QuaternionEKF_Reset(void)
{
    IMU_QuaternionEKF_Reset_Instance(&QEKF_INS);
}

float Get_Pitch(void)
{
    return IMU_QuaternionEKF_GetPitch(&QEKF_INS);
}

float Get_Roll(void)
{
    return IMU_QuaternionEKF_GetRoll(&QEKF_INS);
}

float Get_Yaw(void)
{
    return IMU_QuaternionEKF_GetYaw(&QEKF_INS);
}

void Get_Quaternion(float q[4])
{
    IMU_QuaternionEKF_GetQuaternion(&QEKF_INS, q);
}

void Get_AngularVelocity(float gyro[3])
{
    IMU_QuaternionEKF_GetAngularVelocity(&QEKF_INS, gyro);
}

void Get_FilteredAngularVelocity(float gyro[3])
{
    IMU_QuaternionEKF_GetFilteredAngularVelocity(&QEKF_INS, gyro);
}

void Get_AngularAcceleration(float gyro_accel[3])
{
    IMU_QuaternionEKF_GetAngularAcceleration(&QEKF_INS, gyro_accel);
}

static void resetFilterState(QEKF_INS_t *ins, int filters_ready)
{
    KalmanFilter_t attitude = ins->IMU_QuaternionEKF;
    KalmanFilter_t gyro_accel = ins->GyroAccelKF;

    memset(ins, 0, sizeof(*ins));

    if (filters_ready) {
        ins->IMU_QuaternionEKF = attitude;
        ins->GyroAccelKF = gyro_accel;
        Kalman_Filter_Reset(&ins->IMU_QuaternionEKF, 6, 0, 3);
        Kalman_Filter_Reset(&ins->GyroAccelKF, 3, 0, 3);
    } else {
        Kalman_Filter_Init(&ins->IMU_QuaternionEKF, 6, 0, 3);
        Kalman_Filter_Init(&ins->GyroAccelKF, 3, 0, 3);
    }

    Matrix_Init(&ins->ChiSquare, 1, 1, ins->ChiSquare_Data);
}

static void configureFilters(QEKF_INS_t *ins)
{
    memcpy(ins->IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));
    memcpy(ins->IMU_QuaternionEKF.P_data, IMU_QuaternionEKF_P_Const, sizeof(IMU_QuaternionEKF_P_Const));

    memcpy(ins->GyroAccelKF.F_data, IMU_GyroAccelKF_F, sizeof(IMU_GyroAccelKF_F));
    memcpy(ins->GyroAccelKF.H_data, IMU_GyroAccelKF_H, sizeof(IMU_GyroAccelKF_H));
    memcpy(ins->GyroAccelKF.P_data, IMU_GyroAccelKF_P_Init, sizeof(IMU_GyroAccelKF_P_Init));
    ins->GyroAccelKF.Q_data[0] = GYRO_ACCEL_KF_Q;
    ins->GyroAccelKF.Q_data[4] = GYRO_ACCEL_KF_Q;
    ins->GyroAccelKF.Q_data[8] = GYRO_ACCEL_KF_Q;
    ins->GyroAccelKF.R_data[0] = GYRO_ACCEL_KF_R;
    ins->GyroAccelKF.R_data[4] = GYRO_ACCEL_KF_R;
    ins->GyroAccelKF.R_data[8] = GYRO_ACCEL_KF_R;

    ins->IMU_QuaternionEKF.user_data = ins;
    ins->GyroAccelKF.user_data = ins;
    ins->IMU_QuaternionEKF.User_Func0_f = IMU_QuaternionEKF_Observe;
    ins->IMU_QuaternionEKF.User_Func1_f = IMU_QuaternionEKF_F_Linearization_P_Fading;
    ins->IMU_QuaternionEKF.User_Func2_f = IMU_QuaternionEKF_SetH;
    ins->IMU_QuaternionEKF.User_Func3_f = IMU_QuaternionEKF_xhatUpdate;
    ins->IMU_QuaternionEKF.SkipEq3 = TRUE;
    ins->IMU_QuaternionEKF.SkipEq4 = TRUE;
}

static void IMU_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t *kf)
{
    QEKF_INS_t *ins = ownerFromFilter(kf);
    volatile float q0, q1, q2, q3;
    volatile float qInvNorm;

    if (ins == NULL) {
        return;
    }

    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];

    qInvNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    for (uint8_t i = 0; i < 4U; i++) {
        kf->xhatminus_data[i] *= qInvNorm;
    }

    kf->F_data[4] = q1 * ins->dt / 2.0f;
    kf->F_data[5] = q2 * ins->dt / 2.0f;

    kf->F_data[10] = -q0 * ins->dt / 2.0f;
    kf->F_data[11] = q3 * ins->dt / 2.0f;

    kf->F_data[16] = -q3 * ins->dt / 2.0f;
    kf->F_data[17] = -q0 * ins->dt / 2.0f;

    kf->F_data[22] = q2 * ins->dt / 2.0f;
    kf->F_data[23] = -q1 * ins->dt / 2.0f;

    kf->P_data[28] /= ins->lambda;
    kf->P_data[35] /= ins->lambda;

    if (kf->P_data[28] > 10000.0f) {
        kf->P_data[28] = 10000.0f;
    }
    if (kf->P_data[35] > 10000.0f) {
        kf->P_data[35] = 10000.0f;
    }
}

static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf)
{
    volatile float doubleq0, doubleq1, doubleq2, doubleq3;

    doubleq0 = 2.0f * kf->xhatminus_data[0];
    doubleq1 = 2.0f * kf->xhatminus_data[1];
    doubleq2 = 2.0f * kf->xhatminus_data[2];
    doubleq3 = 2.0f * kf->xhatminus_data[3];

    memset(kf->H_data, 0, sizeof_float * kf->zSize * kf->xhatSize);

    kf->H_data[0] = -doubleq2;
    kf->H_data[1] = doubleq3;
    kf->H_data[2] = -doubleq0;
    kf->H_data[3] = doubleq1;

    kf->H_data[6] = doubleq1;
    kf->H_data[7] = doubleq0;
    kf->H_data[8] = doubleq3;
    kf->H_data[9] = doubleq2;

    kf->H_data[12] = doubleq0;
    kf->H_data[13] = -doubleq1;
    kf->H_data[14] = -doubleq2;
    kf->H_data[15] = doubleq3;
}

static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf)
{
    QEKF_INS_t *ins = ownerFromFilter(kf);
    volatile float q0, q1, q2, q3;

    if (ins == NULL) {
        return;
    }

    kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT);
    kf->temp_matrix.numRows = kf->H.numRows;
    kf->temp_matrix.numCols = kf->Pminus.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix);
    kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
    kf->temp_matrix1.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1);
    kf->S.numRows = kf->R.numRows;
    kf->S.numCols = kf->R.numCols;
    kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S);
    kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1);

    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];

    kf->temp_vector.numRows = kf->H.numRows;
    kf->temp_vector.numCols = 1;
    kf->temp_vector_data[0] = 2.0f * (q1 * q3 - q0 * q2);
    kf->temp_vector_data[1] = 2.0f * (q0 * q1 + q2 * q3);
    kf->temp_vector_data[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    for (uint8_t i = 0U; i < 3U; i++) {
        ins->OrientationCosine[i] = cosf(fabsf(kf->temp_vector_data[i]));
    }

    kf->temp_vector1.numRows = kf->z.numRows;
    kf->temp_vector1.numCols = 1;
    kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1);

    kf->temp_matrix.numRows = kf->temp_vector1.numRows;
    kf->temp_matrix.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix1, &kf->temp_vector1, &kf->temp_matrix);
    kf->temp_vector.numRows = 1;
    kf->temp_vector.numCols = kf->temp_vector1.numRows;
    kf->MatStatus = Matrix_Transpose(&kf->temp_vector1, &kf->temp_vector);
    kf->MatStatus = Matrix_Multiply(&kf->temp_vector, &kf->temp_matrix, &ins->ChiSquare);
    chiSquare = ins->ChiSquare_Data[0];

    if (ins->ChiSquare_Data[0] < 0.5f * ins->ChiSquareTestThreshold) {
        ins->ConvergeFlag = 1U;
    }
    if (ins->ChiSquare_Data[0] > ins->ChiSquareTestThreshold && ins->ConvergeFlag != 0U) {
        if (ins->StableFlag != 0U) {
            ins->ErrorCount++;
        } else {
            ins->ErrorCount = 0U;
        }

        if (ins->ErrorCount > 50U) {
            ins->ConvergeFlag = 0U;
            kf->SkipEq5 = FALSE;
        } else {
            memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);
            memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);
            kf->SkipEq5 = TRUE;
            return;
        }
    } else {
        if (ins->ChiSquare_Data[0] > 0.1f * ins->ChiSquareTestThreshold &&
            ins->ConvergeFlag != 0U) {
            ins->AdaptiveGainScale =
                (ins->ChiSquareTestThreshold - ins->ChiSquare_Data[0]) /
                (0.9f * ins->ChiSquareTestThreshold);
        } else {
            ins->AdaptiveGainScale = 1.0f;
        }
        ins->ErrorCount = 0U;
        kf->SkipEq5 = FALSE;
    }

    kf->temp_matrix.numRows = kf->Pminus.numRows;
    kf->temp_matrix.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix);
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);

    for (uint8_t i = 0U; i < kf->K.numRows * kf->K.numCols; i++) {
        kf->K_data[i] *= ins->AdaptiveGainScale;
    }
    for (uint8_t i = 4U; i < 6U; i++) {
        for (uint8_t j = 0U; j < 3U; j++) {
            kf->K_data[i * 3U + j] *= ins->OrientationCosine[i - 4U] / 1.5707963f;
        }
    }

    kf->temp_vector.numRows = kf->K.numRows;
    kf->temp_vector.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector);

    if (ins->ConvergeFlag != 0U) {
        for (uint8_t i = 4U; i < 6U; i++) {
            if (kf->temp_vector.pData[i] > 1e-2f * ins->dt) {
                kf->temp_vector.pData[i] = 1e-2f * ins->dt;
            }
            if (kf->temp_vector.pData[i] < -1e-2f * ins->dt) {
                kf->temp_vector.pData[i] = -1e-2f * ins->dt;
            }
        }
    }

    kf->temp_vector.pData[3] = 0.0f;
    kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
}

static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf)
{
    QEKF_INS_t *ins = ownerFromFilter(kf);
    if (ins == NULL) {
        return;
    }

    memcpy(ins->P_snapshot, kf->P_data, sizeof(ins->P_snapshot));
    memcpy(ins->K_snapshot, kf->K_data, sizeof(ins->K_snapshot));
    memcpy(ins->H_snapshot, kf->H_data, sizeof(ins->H_snapshot));
}

static void IMU_QuaternionEKF_CalculateFilteredGyro(QEKF_INS_t *ins)
{
    const float dq0 = (ins->q[0] - ins->q_prev[0]) / ins->dt;
    const float dq1 = (ins->q[1] - ins->q_prev[1]) / ins->dt;
    const float dq2 = (ins->q[2] - ins->q_prev[2]) / ins->dt;
    const float dq3 = (ins->q[3] - ins->q_prev[3]) / ins->dt;
    const float q0 = ins->q[0];
    const float q1 = ins->q[1];
    const float q2 = ins->q[2];
    const float q3 = ins->q[3];

    ins->GyroFiltered[0] = 2.0f * (q0 * dq1 - q1 * dq0 - q2 * dq3 + q3 * dq2);
    ins->GyroFiltered[1] = 2.0f * (q0 * dq2 + q1 * dq3 - q2 * dq0 - q3 * dq1);
    ins->GyroFiltered[2] = 2.0f * (q0 * dq3 - q1 * dq2 + q2 * dq1 - q3 * dq0);
}

static QEKF_INS_t *ownerFromFilter(KalmanFilter_t *kf)
{
    return kf != NULL ? (QEKF_INS_t *)kf->user_data : NULL;
}

static float invSqrt(float x)
{
    if (x <= 1e-12f) {
        return 1.0f;
    }
    return 1.0f / sqrtf(x);
}

static float clampf(float value, float low, float high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}
