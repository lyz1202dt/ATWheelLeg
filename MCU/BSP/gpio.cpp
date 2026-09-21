#include "gpio.hpp"

namespace bsp {

HAL_StatusTypeDef GpioPin::init(const GPIO_InitTypeDef &config) const
{
    if (port_ == nullptr || pin_ == 0U) {
        return HAL_ERROR;
    }

    GPIO_InitTypeDef local_config = config;
    local_config.Pin = pin_;
    HAL_GPIO_Init(port_, &local_config);
    return HAL_OK;
}

void GpioPin::deinit() const
{
    if (port_ != nullptr && pin_ != 0U) {
        HAL_GPIO_DeInit(port_, pin_);
    }
}

void GpioPin::write(GPIO_PinState state) const
{
    if (port_ != nullptr && pin_ != 0U) {
        HAL_GPIO_WritePin(port_, pin_, state);
    }
}

void GpioPin::toggle() const
{
    if (port_ != nullptr && pin_ != 0U) {
        HAL_GPIO_TogglePin(port_, pin_);
    }
}

GPIO_PinState GpioPin::read() const
{
    if (port_ == nullptr || pin_ == 0U) {
        return GPIO_PIN_RESET;
    }
    return HAL_GPIO_ReadPin(port_, pin_);
}

} // namespace bsp
