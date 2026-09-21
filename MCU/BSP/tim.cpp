#include "tim.hpp"

namespace bsp {

HAL_StatusTypeDef Timer::start() const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }
    return HAL_TIM_Base_Start(handle_);
}

HAL_StatusTypeDef Timer::stop() const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }
    return HAL_TIM_Base_Stop(handle_);
}

HAL_StatusTypeDef Timer::startIt() const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }
    return HAL_TIM_Base_Start_IT(handle_);
}

HAL_StatusTypeDef Timer::stopIt() const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }
    return HAL_TIM_Base_Stop_IT(handle_);
}

void Timer::setCounter(uint32_t counter) const
{
    if (handle_ != nullptr) {
        __HAL_TIM_SET_COUNTER(handle_, counter);
    }
}

uint32_t Timer::counter() const
{
    if (handle_ == nullptr) {
        return 0U;
    }
    return __HAL_TIM_GET_COUNTER(handle_);
}

void Timer::setAutoreload(uint32_t autoreload) const
{
    if (handle_ != nullptr) {
        __HAL_TIM_SET_AUTORELOAD(handle_, autoreload);
    }
}

uint32_t Timer::autoreload() const
{
    if (handle_ == nullptr) {
        return 0U;
    }
    return __HAL_TIM_GET_AUTORELOAD(handle_);
}

void PwmChannel::bind(TIM_HandleTypeDef &handle, uint32_t channel)
{
    handle_ = &handle;
    channel_ = channel;
}

HAL_StatusTypeDef PwmChannel::start() const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }
    return HAL_TIM_PWM_Start(handle_, channel_);
}

HAL_StatusTypeDef PwmChannel::stop() const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }
    return HAL_TIM_PWM_Stop(handle_, channel_);
}

HAL_StatusTypeDef PwmChannel::startIt() const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }
    return HAL_TIM_PWM_Start_IT(handle_, channel_);
}

HAL_StatusTypeDef PwmChannel::stopIt() const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }
    return HAL_TIM_PWM_Stop_IT(handle_, channel_);
}

HAL_StatusTypeDef PwmChannel::startDma(const uint32_t *data, uint16_t length) const
{
    if (handle_ == nullptr || data == nullptr) {
        return HAL_ERROR;
    }
    return HAL_TIM_PWM_Start_DMA(handle_, channel_, data, length);
}

HAL_StatusTypeDef PwmChannel::stopDma() const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }
    return HAL_TIM_PWM_Stop_DMA(handle_, channel_);
}

void PwmChannel::setPulse(uint32_t pulse) const
{
    if (handle_ != nullptr) {
        __HAL_TIM_SET_COMPARE(handle_, channel_, pulse);
    }
}

uint32_t PwmChannel::pulse() const
{
    if (handle_ == nullptr) {
        return 0U;
    }
    return __HAL_TIM_GET_COMPARE(handle_, channel_);
}

void PwmChannel::setDutyPermille(uint16_t duty_permille) const
{
    if (handle_ == nullptr) {
        return;
    }

    if (duty_permille > 1000U) {
        duty_permille = 1000U;
    }

    const uint32_t compare = (__HAL_TIM_GET_AUTORELOAD(handle_) * duty_permille) / 1000U;
    setPulse(compare);
}

} // namespace bsp
