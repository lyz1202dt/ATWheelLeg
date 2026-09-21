#pragma once

#include "adc.hpp"
#include "bmi088.hpp"
#include "fdcan.hpp"
#include "gpio.hpp"
#include "spi.hpp"
#include "tim.hpp"
#include "usart.hpp"

namespace bsp::hardware {

extern GpioPin bmi088_accel_cs;
extern GpioPin bmi088_gyro_cs;
extern GpioPin bmi088_accel_int;
extern GpioPin bmi088_gyro_int;

extern SpiBus spi2;
extern SpiBus spi6;

extern UartPort uart2;
extern UartPort uart3;

extern FdcanBus fdcan1;

extern AdcChannel adc3;

extern Timer tim3;
extern Timer tim13;
extern PwmChannel tim3_ch4;

extern bmi088::Bmi088 imu;

HAL_StatusTypeDef init();

} // namespace bsp::hardware

extern "C" void HardwareInit(void);
