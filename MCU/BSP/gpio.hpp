#pragma once

#include "stm32h7xx_hal.h"

namespace bsp {

class GpioPin {
public:
    constexpr GpioPin(GPIO_TypeDef *port, uint16_t pin) : port_(port), pin_(pin) {}

    HAL_StatusTypeDef init(const GPIO_InitTypeDef &config) const;
    void deinit() const;

    void write(GPIO_PinState state) const;
    void toggle() const;
    GPIO_PinState read() const;
private:
    GPIO_TypeDef *port_ = nullptr;
    uint16_t pin_ = 0U;
};

} // namespace bsp
