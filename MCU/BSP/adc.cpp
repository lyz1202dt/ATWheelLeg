#include "adc.hpp"

#include <unordered_map>
#include <utility>

namespace {

struct AdcState {
    std::function<void()> dma_cplt;
    std::function<void()> injected_cplt;
};

std::unordered_map<ADC_HandleTypeDef *, AdcState> &adcStates()
{
    static std::unordered_map<ADC_HandleTypeDef *, AdcState> states;
    return states;
}

void dispatchDmaCplt(ADC_HandleTypeDef *handle)
{
    const auto it = adcStates().find(handle);
    if (it != adcStates().end() && it->second.dma_cplt) {
        it->second.dma_cplt();
    }
}

void dispatchInjectedCplt(ADC_HandleTypeDef *handle)
{
    const auto it = adcStates().find(handle);
    if (it != adcStates().end() && it->second.injected_cplt) {
        it->second.injected_cplt();
    }
}

} // namespace

namespace bsp {

HAL_StatusTypeDef AdcChannel::startDma(uint32_t *data, uint32_t length, Callback cplt) const
{
    if (handle_ == nullptr || data == nullptr || length == 0U) {
        return HAL_ERROR;
    }

    AdcState &state = adcStates()[handle_];
    state.dma_cplt = std::move(cplt);

    const HAL_StatusTypeDef status = HAL_ADC_Start_DMA(handle_, data, length);
    if (status != HAL_OK) {
        state.dma_cplt = nullptr;
    }
    return status;
}

HAL_StatusTypeDef AdcChannel::stopDma() const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }

    const HAL_StatusTypeDef status = HAL_ADC_Stop_DMA(handle_);
    if (status == HAL_OK) {
        adcStates()[handle_].dma_cplt = nullptr;
    }
    return status;
}

HAL_StatusTypeDef AdcChannel::injectedStartIt(Callback cplt) const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }

    AdcState &state = adcStates()[handle_];
    state.injected_cplt = std::move(cplt);

    const HAL_StatusTypeDef status = HAL_ADCEx_InjectedStart_IT(handle_);
    if (status != HAL_OK) {
        state.injected_cplt = nullptr;
    }
    return status;
}

HAL_StatusTypeDef AdcChannel::injectedStopIt() const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }
    return HAL_ADCEx_InjectedStop_IT(handle_);
}

uint32_t AdcChannel::injectedValue(uint32_t rank) const
{
    if (handle_ == nullptr) {
        return 0U;
    }
    return HAL_ADCEx_InjectedGetValue(handle_, rank);
}

} // namespace bsp

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    dispatchDmaCplt(hadc);
}

extern "C" void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    dispatchInjectedCplt(hadc);
}
