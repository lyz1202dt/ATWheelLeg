#pragma once

#include "stm32h7xx_hal.h"

namespace bsp {

class Timer {
public:
    explicit constexpr Timer(TIM_HandleTypeDef &handle) : handle_(&handle) {}

    HAL_StatusTypeDef start() const;
    HAL_StatusTypeDef stop() const;
    HAL_StatusTypeDef startIt() const;
    HAL_StatusTypeDef stopIt() const;

    void setCounter(uint32_t counter) const;
    uint32_t counter() const;
    void setAutoreload(uint32_t autoreload) const;
    uint32_t autoreload() const;

private:
    TIM_HandleTypeDef *handle_ = nullptr;
};

class PwmChannel {
public:
    constexpr PwmChannel() = default;
    constexpr PwmChannel(TIM_HandleTypeDef &handle, uint32_t channel)
        : handle_(&handle), channel_(channel)
    {
    }

    void bind(TIM_HandleTypeDef &handle, uint32_t channel);
    TIM_HandleTypeDef *native() const { return handle_; }
    uint32_t channel() const { return channel_; }
    bool valid() const { return handle_ != nullptr; }

    HAL_StatusTypeDef start() const;
    HAL_StatusTypeDef stop() const;
    HAL_StatusTypeDef startIt() const;
    HAL_StatusTypeDef stopIt() const;
    HAL_StatusTypeDef startDma(const uint32_t *data, uint16_t length) const;
    HAL_StatusTypeDef stopDma() const;

    void setPulse(uint32_t pulse) const;
    uint32_t pulse() const;
    void setDutyPermille(uint16_t duty_permille) const;

private:
    TIM_HandleTypeDef *handle_ = nullptr;
    uint32_t channel_ = TIM_CHANNEL_1;
};

} // namespace bsp
