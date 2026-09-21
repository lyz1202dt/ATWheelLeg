#include "app_mian.hpp"
#include "bmi088_imu.hpp"
#include "controller.hpp"
#include "hardware.hpp"

#include "motorbase.hpp"

#include <FreeRTOS.h>
#include <task.h>

#include <array>
#include <cstdint>
#include <string>

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
Controller controller_instance{imu, lf_motor, rf_motor, lb_motor, rb_motor, lw_motor, rw_motor};
Controller* controller = &controller_instance;

void app_main(void) {
    if (bsp::hardware::init() != HAL_OK) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        return;
    }

    bmi088_imu.init();

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
    Eigen::Quaternionf q         = Eigen::Quaternionf::Identity();
    Eigen::Vector3d angular      = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    TickType_t last_wake_time = xTaskGetTickCount();
    for (;;) {
        (void)imu->update(q, angular, acceleration, 0.001F);
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
    controller->imu = imu;
    controller->lf  = lf_motor;
    controller->rf  = rf_motor;
    controller->lb  = lb_motor;
    controller->rb  = rb_motor;
    controller->lw  = lw_motor;
    controller->rw  = rw_motor;
    TickType_t last_wake_time = xTaskGetTickCount();
    for (;;) {
        (void)controller->update(0.002f);
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(2));
    }
}

void LqrTask(void* param) {
    (void)param;
    uint32_t consumed_request = 0U;

    for (;;) {
        const uint32_t request = lqr_gain_update_request;
        if (controller != nullptr && request != consumed_request) {
            std::array<double, 6> q_diag{};
            std::array<double, 2> r_diag{};
            for (uint32_t index = 0U; index < 6U; ++index) {
                q_diag[index] = static_cast<double>(lqr_q_diag[index]);
            }
            for (uint32_t index = 0U; index < 2U; ++index) {
                r_diag[index] = static_cast<double>(lqr_r_diag[index]);
            }

            std::string error;
            (void)controller->update_lqr_gain(q_diag, r_diag, error);
            consumed_request = request;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
