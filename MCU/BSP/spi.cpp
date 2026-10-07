#include "spi.hpp"

#include "task.h"

namespace bsp {

SpiBus::SpiBus(SPI_HandleTypeDef& handle, uint32_t queue_length)
    : handle_(&handle)
{
    if (queue_length != 0U) {
        queue_ = xQueueCreate(static_cast<UBaseType_t>(queue_length),
                              sizeof(SpiTransferEvent));
    }

    if (queue_ != nullptr) {
        instances()[handle_] = this;
    }
}

SpiBus::~SpiBus()
{
    instances().erase(handle_);
    if (queue_ != nullptr) {
        vQueueDelete(queue_);
        queue_ = nullptr;
    }
}

std::unordered_map<SPI_HandleTypeDef*, SpiBus*>& SpiBus::instances()
{
    static std::unordered_map<SPI_HandleTypeDef*, SpiBus*> instance_map;
    return instance_map;
}

SpiBus* SpiBus::find(SPI_HandleTypeDef* hspi)
{
    const auto it = instances().find(hspi);
    return it == instances().end() ? nullptr : it->second;
}

bool SpiBus::inIsr() const
{
    return __get_IPSR() != 0U;
}

bool SpiBus::transferValid(const SpiTransferEvent& event) const
{
    return event.size != 0U && (event.tx_data != nullptr || event.rx_data != nullptr);
}

HAL_StatusTypeDef SpiBus::applyHardwareParams(const SpiTransferEvent& event) const
{
    const SpiHardwareParams& hardware = event.hardware;
    if (hardware.data_size == 0U && hardware.clk_polarity == 0U &&
        hardware.clk_phase == 0U && hardware.first_bit == 0U) {
        return HAL_OK;
    }

    if (hardware.data_size != 0U) {
        handle_->Init.DataSize = hardware.data_size;
    }
    if (hardware.clk_polarity != 0U) {
        handle_->Init.CLKPolarity = hardware.clk_polarity;
    }
    if (hardware.clk_phase != 0U) {
        handle_->Init.CLKPhase = hardware.clk_phase;
    }
    if (hardware.first_bit != 0U) {
        handle_->Init.FirstBit = hardware.first_bit;
    }

    return HAL_SPI_Init(handle_);
}

HAL_StatusTypeDef SpiBus::startHalTransfer(SpiTransferEvent& event) const
{
    const auto* tx_data = static_cast<const uint8_t*>(event.tx_data);
    auto* rx_data = static_cast<uint8_t*>(event.rx_data);

    if (tx_data != nullptr && rx_data != nullptr) {
        event.type = SpiTransferType::TransmitReceive;
        return event.mode == SpiTransferMode::Dma
                   ? HAL_SPI_TransmitReceive_DMA(handle_, tx_data, rx_data, event.size)
                   : HAL_SPI_TransmitReceive_IT(handle_, tx_data, rx_data, event.size);
    }
    if (tx_data != nullptr) {
        event.type = SpiTransferType::Transmit;
        return event.mode == SpiTransferMode::Dma
                   ? HAL_SPI_Transmit_DMA(handle_, tx_data, event.size)
                   : HAL_SPI_Transmit_IT(handle_, tx_data, event.size);
    }

    event.type = SpiTransferType::Receive;
    return event.mode == SpiTransferMode::Dma
               ? HAL_SPI_Receive_DMA(handle_, rx_data, event.size)
               : HAL_SPI_Receive_IT(handle_, rx_data, event.size);
}

bool SpiBus::enqueueTransfer(SpiTransferEvent& event)
{
    if (!valid() || !transferValid(event)) {
        return false;
    }

    event.handle = handle_;
    event.status = HAL_OK;
    event.error_code = HAL_SPI_ERROR_NONE;

    if (inIsr()) {
        BaseType_t higher_priority_task_woken = pdFALSE;
        if (xQueueSendFromISR(queue_, &event, &higher_priority_task_woken) != pdPASS) {
            return false;
        }
        (void)startNextTransfer(true, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
        return true;
    }

    if (xQueueSend(queue_, &event, 0U) != pdPASS) {
        return false;
    }

    (void)startNextTransfer(false);
    return true;
}

bool SpiBus::takeNextTransfer(bool from_isr, BaseType_t* higher_priority_task_woken)
{
    if (active_) {
        return false;
    }

    bool has_transfer = false;
    if (from_isr) {
        const UBaseType_t mask = portSET_INTERRUPT_MASK_FROM_ISR();
        has_transfer =
            xQueueReceiveFromISR(queue_, &active_event_, higher_priority_task_woken) == pdPASS;
        active_ = has_transfer;
        portCLEAR_INTERRUPT_MASK_FROM_ISR(mask);
    } else {
        taskENTER_CRITICAL();
        has_transfer = xQueueReceive(queue_, &active_event_, 0U) == pdPASS;
        active_ = has_transfer;
        taskEXIT_CRITICAL();
    }

    return has_transfer;
}

bool SpiBus::startNextTransfer(bool from_isr, BaseType_t* higher_priority_task_woken)
{
    if (!takeNextTransfer(from_isr, higher_priority_task_woken)) {
        return false;
    }

    if (!transferValid(active_event_)) {
        completeTransfer(HAL_ERROR, from_isr, true);
        return false;
    }

    active_event_.status = HAL_BUSY;
    active_event_.error_code = HAL_SPI_ERROR_NONE;
    if (active_event_.before_cb != nullptr) {
        active_event_.before_cb(active_event_);
    }

    HAL_StatusTypeDef status = applyHardwareParams(active_event_);
    if (status == HAL_OK) {
        status = startHalTransfer(active_event_);
    }
    if (status != HAL_OK) {
        completeTransfer(status, from_isr, true);
        return false;
    }

    return true;
}

void SpiBus::clearActiveTransfer(bool from_isr)
{
    if (from_isr) {
        const UBaseType_t mask = portSET_INTERRUPT_MASK_FROM_ISR();
        active_ = false;
        portCLEAR_INTERRUPT_MASK_FROM_ISR(mask);
    } else {
        taskENTER_CRITICAL();
        active_ = false;
        taskEXIT_CRITICAL();
    }
}

void SpiBus::completeTransfer(HAL_StatusTypeDef status, bool from_isr, bool start_next)
{
    if (active_) {
        active_event_.status = status;
        active_event_.error_code = HAL_SPI_GetError(handle_);
        SpiTransferEvent finished_event = active_event_;
        clearActiveTransfer(from_isr);
        if (finished_event.after_cb != nullptr) {
            finished_event.after_cb(finished_event);
        }
    }

    BaseType_t higher_priority_task_woken = pdFALSE;
    if (start_next) {
        (void)startNextTransfer(from_isr, &higher_priority_task_woken);
    }

    if (from_isr) {
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

bool SpiBus::init()
{
    if (!valid()) {
        return false;
    }

    active_ = false;
    xQueueReset(queue_);

#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
    bool registered = true;

    if (HAL_SPI_RegisterCallback(handle_, HAL_SPI_TX_COMPLETE_CB_ID,
                                 &SpiBus::txCpltCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_SPI_RegisterCallback(handle_, HAL_SPI_RX_COMPLETE_CB_ID,
                                 &SpiBus::rxCpltCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_SPI_RegisterCallback(handle_, HAL_SPI_TX_RX_COMPLETE_CB_ID,
                                 &SpiBus::txRxCpltCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_SPI_RegisterCallback(handle_, HAL_SPI_TX_HALF_COMPLETE_CB_ID,
                                 &SpiBus::txHalfCpltCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_SPI_RegisterCallback(handle_, HAL_SPI_RX_HALF_COMPLETE_CB_ID,
                                 &SpiBus::rxHalfCpltCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_SPI_RegisterCallback(handle_, HAL_SPI_TX_RX_HALF_COMPLETE_CB_ID,
                                 &SpiBus::txRxHalfCpltCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_SPI_RegisterCallback(handle_, HAL_SPI_ERROR_CB_ID,
                                 &SpiBus::errorCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_SPI_RegisterCallback(handle_, HAL_SPI_ABORT_CB_ID,
                                 &SpiBus::abortCpltCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_SPI_RegisterCallback(handle_, HAL_SPI_SUSPEND_CB_ID,
                                 &SpiBus::suspendCallback) != HAL_OK) {
        registered = false;
    }

    return registered;
#else
    return false;
#endif
}

bool SpiBus::submit(SpiTransferEvent& event)
{
    return enqueueTransfer(event);
}

bool SpiBus::transferIt(SpiTransferEvent& event)
{
    event.mode = SpiTransferMode::Interrupt;
    return submit(event);
}

bool SpiBus::transferDma(SpiTransferEvent& event)
{
    event.mode = SpiTransferMode::Dma;
    return submit(event);
}

bool SpiBus::abort()
{
    if (!valid()) {
        return false;
    }

    const HAL_StatusTypeDef status = HAL_SPI_Abort(handle_);
    if (queue_ != nullptr) {
        xQueueReset(queue_);
    }
    completeTransfer(status == HAL_OK ? HAL_ERROR : status, false, false);
    return status == HAL_OK;
}

void SpiBus::txCpltCallback(SPI_HandleTypeDef* hspi)
{
    SpiBus* bus = find(hspi);
    if (bus != nullptr) {
        bus->completeTransfer(HAL_OK, true, true);
    }
}

void SpiBus::rxCpltCallback(SPI_HandleTypeDef* hspi)
{
    SpiBus* bus = find(hspi);
    if (bus != nullptr) {
        bus->completeTransfer(HAL_OK, true, true);
    }
}

void SpiBus::txRxCpltCallback(SPI_HandleTypeDef* hspi)
{
    SpiBus* bus = find(hspi);
    if (bus != nullptr) {
        bus->completeTransfer(HAL_OK, true, true);
    }
}

void SpiBus::txHalfCpltCallback(SPI_HandleTypeDef* hspi)
{
    (void)hspi;
}

void SpiBus::rxHalfCpltCallback(SPI_HandleTypeDef* hspi)
{
    (void)hspi;
}

void SpiBus::txRxHalfCpltCallback(SPI_HandleTypeDef* hspi)
{
    (void)hspi;
}

void SpiBus::errorCallback(SPI_HandleTypeDef* hspi)
{
    SpiBus* bus = find(hspi);
    if (bus != nullptr) {
        bus->completeTransfer(HAL_ERROR, true, true);
    }
}

void SpiBus::abortCpltCallback(SPI_HandleTypeDef* hspi)
{
    SpiBus* bus = find(hspi);
    if (bus != nullptr) {
        bus->completeTransfer(HAL_ERROR, true, true);
    }
}

void SpiBus::suspendCallback(SPI_HandleTypeDef* hspi)
{
    (void)hspi;
}

} // namespace bsp
