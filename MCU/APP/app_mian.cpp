#include "app_mian.hpp"
#include "hardware.hpp"

#include "motorbase.hpp"
#include "wheel_leg_estimater.hpp"
#include "wheel_leg_lqr_controller.hpp"

#include <FreeRTOS.h>
#include <task.h>

TaskHandle_t imu_task_handle;
TaskHandle_t motor_task_handle;
TaskHandle_t control_task_handle;
TaskHandle_t test_task_handle;

void TestTask(void* param);
void IMUTask(void* param);
void MotorTask(void* param);
void ControlTask(void* param);

//app_main负责初始化
Motor* lf_motor, *lb_motor, *rf_motor, *rb_motor,*lw_motor,*rw_motor;
Controller* controller;

void app_main(void) {
    bsp::hardware::init();
    xTaskCreate(IMUTask, "imu_task", 1024, nullptr, 4, &imu_task_handle);
    xTaskCreate(MotorTask, "motor_task", 512, nullptr, 4, &motor_task_handle);
    xTaskCreate(ControlTask, "control_task", 1024, nullptr, 3, &control_task_handle);
    xTaskCreate(TestTask, "test_task", 512, nullptr, 1, &test_task_handle);
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void TestTask(void* param) {
    while (1) {

    }
}

void IMUTask(void* param) {
    while (1) {

    }
}
void MotorTask(void* param) {
    while (1) {

    }
}
void ControlTask(void* param) {
    while (1) {

    }
}
