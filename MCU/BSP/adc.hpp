#pragma once

#include "stm32h7xx_hal.h"

#include <functional>

namespace bsp {

class AdcChannel {
public:
    using Callback = std::function<void()>;

    constexpr AdcChannel() = default;
    explicit constexpr AdcChannel(ADC_HandleTypeDef &handle) : handle_(&handle) {}

    // 规则组：软件触发 + DMA。数据写入 data 数组，传输完成后调用 cplt 回调。
    HAL_StatusTypeDef startDma(uint32_t *data, uint32_t length, Callback cplt = nullptr) const;
    HAL_StatusTypeDef stopDma() const;

    // 注入组：软件触发 + 中断。注入序列转换完成后调用 cplt 回调（回调内用 injectedValue 读取结果）。
    HAL_StatusTypeDef injectedStartIt(Callback cplt = nullptr) const;
    HAL_StatusTypeDef injectedStopIt() const;
    uint32_t injectedValue(uint32_t rank) const;

private:
    ADC_HandleTypeDef *handle_ = nullptr;
};

} // namespace bsp
