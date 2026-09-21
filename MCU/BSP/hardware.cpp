#include "hardware.hpp"

#include "adc.h"
#include "fdcan.h"
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

namespace bsp::hardware {

GpioPin bmi088_accel_cs{ACC_CS_GPIO_Port, ACC_CS_Pin};
GpioPin bmi088_gyro_cs{GYRO_CS_GPIO_Port, GYRO_CS_Pin};
GpioPin bmi088_accel_int{ACC_INT_GPIO_Port, ACC_INT_Pin};
GpioPin bmi088_gyro_int{GYRO_INT_GPIO_Port, GYRO_INT_Pin};

SpiBus spi2{hspi2};
SpiBus spi6{hspi6};

UartPort uart2{huart2};
UartPort uart3{huart3};

FdcanBus fdcan1{hfdcan1};

AdcChannel adc3{hadc3};

Timer tim3{htim3};
Timer tim13{htim13};
PwmChannel tim3_ch4{htim3, TIM_CHANNEL_4};

bmi088::Bmi088 imu{spi2, bmi088_accel_cs, bmi088_gyro_cs, tim3_ch4, adc3};

HAL_StatusTypeDef init()
{
    bmi088_accel_cs.write(GPIO_PIN_SET);
    bmi088_gyro_cs.write(GPIO_PIN_SET);

    if (!spi2.init() || !spi6.init()) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status = fdcan1.configAcceptAllFilters();
    if (status != HAL_OK) {
        return status;
    }

    return fdcan1.start(FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 1U);
}

} // namespace bsp::hardware

extern "C" void HardwareInit(void)
{
    if (bsp::hardware::init() != HAL_OK) {
        Error_Handler();
    }
}
