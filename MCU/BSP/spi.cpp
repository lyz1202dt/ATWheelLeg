#include "spi.hpp"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

namespace bsp {

SpiBus::SpiBus(SPI_HandleTypeDef& handle, uint32_t queue_length)
    : handle_(&handle)
{
    if (queue_length != 0U) {
        queue_ = xQueueCreate(static_cast<UBaseType_t>(queue_length),
                              sizeof(QueuedTransfer));
    }

    if (handle_ != nullptr && queue_ != nullptr) {
        instances()[handle_] = this;
    }
}

SpiBus::~SpiBus()
{
    if (handle_ != nullptr) {
        instances().erase(handle_);
    }
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
    if (hspi == nullptr) {
        return nullptr;
    }

    auto& instance_map = instances();
    const auto it = instance_map.find(hspi);
    return it == instance_map.end() ? nullptr : it->second;
}

bool SpiBus::inIsr() const
{
    return __get_IPSR() != 0U;
}

bool SpiBus::transferValid(const SPITransfer& transfer) const
{
    return transfer.size != 0U && (transfer.tx_data != nullptr || transfer.rx_data != nullptr);
}

HAL_StatusTypeDef SpiBus::startHalTransfer(SPITransfer& transfer, TransferMode mode) const
{
    const auto* tx_data = static_cast<const uint8_t*>(transfer.tx_data);
    auto* rx_data = static_cast<uint8_t*>(transfer.rx_data);

    if (tx_data != nullptr && rx_data != nullptr) {
        return mode == TransferMode::Dma
                   ? HAL_SPI_TransmitReceive_DMA(handle_, tx_data, rx_data, transfer.size)
                   : HAL_SPI_TransmitReceive_IT(handle_, tx_data, rx_data, transfer.size);
    }
    if (tx_data != nullptr) {
        return mode == TransferMode::Dma
                   ? HAL_SPI_Transmit_DMA(handle_, tx_data, transfer.size)
                   : HAL_SPI_Transmit_IT(handle_, tx_data, transfer.size);
    }
    if (rx_data != nullptr) {
        return mode == TransferMode::Dma
                   ? HAL_SPI_Receive_DMA(handle_, rx_data, transfer.size)
                   : HAL_SPI_Receive_IT(handle_, rx_data, transfer.size);
    }

    return HAL_ERROR;
}

SPITransfer* SpiBus::clearActiveTransfer(bool from_isr)
{
    SPITransfer* transfer = nullptr;
    if (from_isr) {
        const UBaseType_t mask = portSET_INTERRUPT_MASK_FROM_ISR();
        transfer = active_transfer_;
        active_transfer_ = nullptr;
        portCLEAR_INTERRUPT_MASK_FROM_ISR(mask);
    } else {
        taskENTER_CRITICAL();
        transfer = active_transfer_;
        active_transfer_ = nullptr;
        taskEXIT_CRITICAL();
    }

    return transfer;
}

bool SpiBus::takeNextTransfer(bool from_isr,
                              QueuedTransfer& queued,
                              BaseType_t* higher_priority_task_woken)
{
    if (queue_ == nullptr) {
        return false;
    }

    bool has_transfer = false;
    if (from_isr) {
        const UBaseType_t mask = portSET_INTERRUPT_MASK_FROM_ISR();
        if (active_transfer_ == nullptr &&
            xQueueReceiveFromISR(queue_, &queued, higher_priority_task_woken) == pdPASS) {
            active_transfer_ = queued.transfer;
            active_mode_ = queued.mode;
            has_transfer = true;
        }
        portCLEAR_INTERRUPT_MASK_FROM_ISR(mask);
    } else {
        taskENTER_CRITICAL();
        if (active_transfer_ == nullptr &&
            xQueueReceive(queue_, &queued, 0U) == pdPASS) {
            active_transfer_ = queued.transfer;
            active_mode_ = queued.mode;
            has_transfer = true;
        }
        taskEXIT_CRITICAL();
    }

    return has_transfer;
}

bool SpiBus::startNextTransfer(bool from_isr, BaseType_t* higher_priority_task_woken)
{
    QueuedTransfer queued;
    if (!takeNextTransfer(from_isr, queued, higher_priority_task_woken)) {
        return false;
    }

    SPITransfer* transfer = queued.transfer;
    if (transfer == nullptr || !transferValid(*transfer)) {
        completeTransfer(HAL_ERROR, from_isr, true);
        return false;
    }

    transfer->status = HAL_BUSY;
    transfer->error_code = HAL_SPI_ERROR_NONE;

    if (transfer->pre_transmit_cb) {
        transfer->pre_transmit_cb(transfer);
    }

    const HAL_StatusTypeDef status = startHalTransfer(*transfer, queued.mode);
    if (status != HAL_OK) {
        completeTransfer(status, from_isr, true);
        return false;
    }

    return true;
}

void SpiBus::completeTransfer(HAL_StatusTypeDef status, bool from_isr, bool start_next)
{
    SPITransfer* transfer = clearActiveTransfer(from_isr);
    if (transfer != nullptr) {
        transfer->status = status;
        transfer->error_code = HAL_SPI_GetError(handle_);
        if (transfer->transmited_cb) {
            transfer->transmited_cb(transfer);
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

bool SpiBus::enqueueTransfer(SPITransfer& transfer, TransferMode mode)
{
    if (!valid() || !transferValid(transfer)) {
        return false;
    }

    const QueuedTransfer queued{&transfer, mode};
    if (inIsr()) {
        BaseType_t higher_priority_task_woken = pdFALSE;
        if (xQueueSendFromISR(queue_, &queued, &higher_priority_task_woken) != pdPASS) {
            return false;
        }

        (void)startNextTransfer(true, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
        return true;
    }

    if (xQueueSend(queue_, &queued, 0U) != pdPASS) {
        return false;
    }

    (void)startNextTransfer(false);
    return true;
}

bool SpiBus::init()
{
    if (!valid()) {
        return false;
    }

    active_transfer_ = nullptr;
    active_mode_ = TransferMode::Interrupt;
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

    if (!registered) {
        instances().erase(handle_);
        return false;
    }

    return true;
#else
    instances().erase(handle_);
    return false;
#endif
}

bool SpiBus::transferIt(SPITransfer& transfer)
{
    return enqueueTransfer(transfer, TransferMode::Interrupt);
}

bool SpiBus::transferDma(SPITransfer& transfer)
{
    return enqueueTransfer(transfer, TransferMode::Dma);
}

bool SpiBus::abort()
{
    if (!valid()) {
        return false;
    }

    const HAL_StatusTypeDef status = HAL_SPI_Abort(handle_);
    SPITransfer* transfer = clearActiveTransfer(false);
    if (queue_ != nullptr) {
        xQueueReset(queue_);
    }

    if (transfer != nullptr) {
        transfer->status = status;
        transfer->error_code = HAL_SPI_GetError(handle_);
        if (transfer->transmited_cb) {
            transfer->transmited_cb(transfer);
        }
    }

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
