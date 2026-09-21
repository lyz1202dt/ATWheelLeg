#include "app_mian.hpp"
#include "bmi088_imu.hpp"
#include "controller.hpp"
#include "hardware.hpp"
#include "k_tab.hpp"

#include "motorbase.hpp"

#include <FreeRTOS.h>
#include <task.h>

#include <vector>

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
    controller->imu = imu;
    controller->lf  = lf_motor;
    controller->rf  = rf_motor;
    controller->lb  = lb_motor;
    controller->rb  = rb_motor;
    controller->lw  = lw_motor;
    controller->rw  = rw_motor;
    const std::vector<double> lengths(
        k_length, k_length + kGainTableLengthCount);
    const std::vector<double> values(
        k_tab, k_tab + kGainTableValueCount);
    std::string err_str;
    if (!controller->set_gain_table(lengths, values, err_str)) {
        vTaskDelete(nullptr);
        return;
    }
    TickType_t last_wake_time = xTaskGetTickCount();
    for (;;) {
        (void)controller->update(0.002f);
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(2));
    }
}
