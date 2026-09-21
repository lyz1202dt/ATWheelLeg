#include "app_mian.hpp"
#include "bmi088_imu.hpp"
#include "controller.hpp"
#include "hardware.hpp"

#include "motorbase.hpp"

#include <FreeRTOS.h>
#include <task.h>

TaskHandle_t imu_task_handle;
TaskHandle_t motor_task_handle;
TaskHandle_t control_task_handle;
TaskHandle_t lqr_task_handle;
TaskHandle_t test_task_handle;

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
IMUBase* imu = &bmi088_imu;
Controller controller_instance{imu, lf_motor, rf_motor, lb_motor, rb_motor, lw_motor, rw_motor};
Controller* controller = &controller_instance;

void app_main(void)
{
    if (bsp::hardware::init() != HAL_OK) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        return;
    }

    bmi088_imu.init();

    (void)xTaskCreate(IMUTask, "imu_task", 1024, nullptr, 4, &imu_task_handle);
    (void)xTaskCreate(MotorTask, "motor_task", 512, nullptr, 4, &motor_task_handle);
    (void)xTaskCreate(ControlTask, "control_task", 1024, nullptr, 3, &control_task_handle);
    // The Hamiltonian eigen solve is intentionally isolated from the control loop.
    (void)xTaskCreate(LqrTask, "lqr_task", 4096, nullptr, 1, &lqr_task_handle);
    (void)xTaskCreate(TestTask, "test_task", 512, nullptr, 1, &test_task_handle);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void TestTask(void* param)
{
    (void)param;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void IMUTask(void* param)
{
    (void)param;
    Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
    Eigen::Vector3d angular = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    for (;;) {
        (void)imu->update(q, angular, acceleration, 0.001F);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void MotorTask(void* param)
{
    (void)param;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void ControlTask(void* param)
{
    (void)param;
    for (;;) {
        controller->imu = imu;
        controller->lf = lf_motor;
        controller->rf = rf_motor;
        controller->lb = lb_motor;
        controller->rb = rb_motor;
        controller->lw = lw_motor;
        controller->rw = rw_motor;
        (void)controller->update(0.001F);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void LqrTask(void* param)
{
    (void)param;
    uint32_t consumed_request = 0U;

    for (;;) {
        const uint32_t request = lqr_gain_update_request;
        if (controller != nullptr && request != consumed_request) {
            float q_diag[6] = {};
            float r_diag[2] = {};
            for (uint32_t index = 0U; index < 6U; ++index) {
                q_diag[index] = lqr_q_diag[index];
            }
            for (uint32_t index = 0U; index < 2U; ++index) {
                r_diag[index] = lqr_r_diag[index];
            }

            Eigen::Matrix<double, 2, 6> gain;
            if (controller->calculate_lqr_gain(q_diag, r_diag, gain) &&
                controller->set_K(gain)) {
                consumed_request = request;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
