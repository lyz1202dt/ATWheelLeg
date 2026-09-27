#include "app_main.hpp"
#include "bmi088_imu.hpp"
#include "controller/controller_6x.hpp"
#include "hardware.hpp"
#include "tools/lqr_calc.hpp"

#include "motorbase.hpp"

#include <FreeRTOS.h>
#include <task.h>

#include <array>
#include <cstdint>
#include <string>

namespace {

Eigen::MatrixXd make_lqr_a()
{
    Eigen::MatrixXd a(6, 6);
    a << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, -13.9285, 0.0, 0.6373, 0.0,
         0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 98.2488, 0.0, 17.6903, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
         0.0, 0.0, 0.0, 44.9938, 0.0, 54.3689;
    return a;
}

Eigen::MatrixXd make_lqr_b()
{
    Eigen::MatrixXd b(6, 2);
    b << 0.0, 0.0,
         15.4595, -3.3942,
         0.0, 0.0,
         -65.5078, 39.5039,
         0.0, 0.0,
         -7.9383, 50.5446;
    return b;
}

} // namespace

TaskHandle_t imu_task_handle;
TaskHandle_t motor_task_handle;
TaskHandle_t control_task_handle;
TaskHandle_t lqr_task_handle;
TaskHandle_t test_task_handle;

extern "C" {
// These variables are intentionally global so a debugger can edit the weights
// and increment lqr_gain_update_request to apply them once.
volatile uint32_t lqr_gain_update_request = 0U;
volatile float lqr_q_diag[6] = {0.3, 3.0, 100.0, 40.0, 600.0, 50.0};
volatile float lqr_r_diag[2] = {4.0, 0.5};
}

void TestTask(void* param);
void IMUTask(void* param);
void MotorTask(void* param);
void ControlTask(void* param);
void LqrTask(void* param);

Bmi088IMU bmi088_imu{bsp::hardware::imu};

Motor* lf_motor = nullptr;
Motor* lb_motor = nullptr;
Motor* rf_motor = nullptr;
Motor* rb_motor = nullptr;
Motor* lw_motor = nullptr;
Motor* rw_motor = nullptr;
IMUBase* imu    = &bmi088_imu;
Controller* lqr_controller=nullptr;
ControllerBase* controller = nullptr;
LQRCalc lqr_calc(make_lqr_a(), make_lqr_b());
LqrGainMatrix lqr_gain = LqrGainMatrix::Zero();

bool provide_lqr_gain(double, LqrGainMatrix& gain)
{
    gain = lqr_gain;
    return gain.allFinite();
}

bool update_lqr_gain()
{
    LQRCalc::Vector q_diag(6);
    LQRCalc::Vector r_diag(2);
    for (Eigen::Index index = 0; index < 6; ++index) {
        q_diag[index] = static_cast<double>(lqr_q_diag[index]);
    }
    for (Eigen::Index index = 0; index < 2; ++index) {
        r_diag[index] = static_cast<double>(lqr_r_diag[index]);
    }

    LQRCalc::Matrix calculated_gain;
    std::string error;
    if (!lqr_calc.calculate(q_diag, r_diag, calculated_gain, error) ||
        calculated_gain.rows() != 2 || calculated_gain.cols() != 6) {
        return false;
    }
    lqr_gain = calculated_gain;
    return lqr_gain.allFinite();
}

void app_main(void) {
    //硬件外设初始化
    if (bsp::hardware::init() != HAL_OK) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        return;
    }
    //陀螺仪初始化
    bmi088_imu.init();

    //创建控制器
    (void)update_lqr_gain();
    lqr_controller = new Controller(
        imu,
        lf_motor,
        rf_motor,
        lb_motor,
        rb_motor,
        lw_motor,
        rw_motor,
        provide_lqr_gain);
    controller=lqr_controller;

    //创建任务
    xTaskCreate(IMUTask, "imu_task", 1024, nullptr, 4, &imu_task_handle);
    xTaskCreate(LqrTask, "lqr_task", 4096, nullptr, 1, &lqr_task_handle);
    vTaskDelay(pdMS_TO_TICKS(500));
    xTaskCreate(MotorTask, "motor_task", 512, nullptr, 4, &motor_task_handle);
    vTaskDelay(pdMS_TO_TICKS(200));
    xTaskCreate(ControlTask, "control_task", 1024, nullptr, 3, &control_task_handle);
    xTaskCreate(TestTask, "test_task", 512, nullptr, 1, &test_task_handle);

    for (;;) {

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void TestTask(void* param) {
    (void)param;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void IMUTask(void* param) {
    (void)param;
    TickType_t last_wake_time = xTaskGetTickCount();
    for (;;) {
        (void)imu->update(0.001F);
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1));
    }
}

void MotorTask(void* param) {
    (void)param;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void ControlTask(void* param) {
    (void)param;
    TickType_t last_wake_time = xTaskGetTickCount();
    for (;;) {
        const uint64_t now_ms =
            static_cast<uint64_t>(xTaskGetTickCount()) * static_cast<uint64_t>(portTICK_PERIOD_MS);
        (void)controller->update(now_ms);
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(2));
    }
}

void LqrTask(void* param) {
    (void)param;
    uint32_t consumed_request = 0U;
    for (;;) {
        const uint32_t request = lqr_gain_update_request;
        if (controller != nullptr && request != consumed_request) {
            if (update_lqr_gain()) {
                consumed_request = request;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
