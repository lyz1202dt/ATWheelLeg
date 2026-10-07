#include "usart.hpp"

#include "task.h"

namespace bsp {

UartPort::UartPort(UART_HandleTypeDef& handle, uint32_t queue_length)
    : handle_(&handle)
{
    if (queue_length != 0U) {
        tx_queue_ = xQueueCreate(static_cast<UBaseType_t>(queue_length),
                                 sizeof(UartTransferEvent));
        rx_queue_ = xQueueCreate(static_cast<UBaseType_t>(queue_length),
                                 sizeof(UartTransferEvent));
    }

    if (tx_queue_ != nullptr && rx_queue_ != nullptr) {
        instances()[handle_] = this;
    }
}

UartPort::~UartPort()
{
    instances().erase(handle_);
    if (tx_queue_ != nullptr) {
        vQueueDelete(tx_queue_);
        tx_queue_ = nullptr;
    }
    if (rx_queue_ != nullptr) {
        vQueueDelete(rx_queue_);
        rx_queue_ = nullptr;
    }
}

std::unordered_map<UART_HandleTypeDef*, UartPort*>& UartPort::instances()
{
    static std::unordered_map<UART_HandleTypeDef*, UartPort*> instance_map;
    return instance_map;
}

UartPort* UartPort::find(UART_HandleTypeDef* huart)
{
    const auto it = instances().find(huart);
    return it == instances().end() ? nullptr : it->second;
}

bool UartPort::inIsr() const
{
    return __get_IPSR() != 0U;
}

bool UartPort::transmitEvent(const UartTransferEvent& event) const
{
    return event.direction == UartTransferDirection::Transmit &&
           event.tx_data != nullptr && event.size != 0U;
}

bool UartPort::receiveEvent(const UartTransferEvent& event) const
{
    return (event.direction == UartTransferDirection::Receive ||
            event.direction == UartTransferDirection::ReceiveToIdle) &&
           event.rx_data != nullptr && event.size != 0U;
}

bool UartPort::receiveKeepAlive(const UartTransferEvent& event) const
{
    if (event.hardware.keep_alive) {
        return true;
    }
    return event.mode == UartTransferMode::Dma && handle_->hdmarx != nullptr &&
           handle_->hdmarx->Init.Mode == DMA_CIRCULAR;
}

HAL_StatusTypeDef UartPort::startHalTransfer(UartTransferEvent& event) const
{
    if (event.direction == UartTransferDirection::Transmit) {
        const auto* data = static_cast<const uint8_t*>(event.tx_data);
        return event.mode == UartTransferMode::Dma
                   ? HAL_UART_Transmit_DMA(handle_, data, event.size)
                   : HAL_UART_Transmit_IT(handle_, data, event.size);
    }

    auto* data = static_cast<uint8_t*>(event.rx_data);
    if (event.direction == UartTransferDirection::ReceiveToIdle) {
        const HAL_StatusTypeDef status =
            HAL_UARTEx_ReceiveToIdle_DMA(handle_, data, event.size);
        if (status == HAL_OK && event.hardware.disable_half_transfer_it &&
            handle_->hdmarx != nullptr) {
            __HAL_DMA_DISABLE_IT(handle_->hdmarx, DMA_IT_HT);
        }
        return status;
    }

    return event.mode == UartTransferMode::Dma
               ? HAL_UART_Receive_DMA(handle_, data, event.size)
               : HAL_UART_Receive_IT(handle_, data, event.size);
}

bool UartPort::enqueueTransfer(UartTransferEvent& event, QueueHandle_t queue)
{
    event.handle = handle_;
    event.status = HAL_OK;
    event.error_code = HAL_UART_ERROR_NONE;
    event.transferred_size = 0U;

    if (inIsr()) {
        BaseType_t higher_priority_task_woken = pdFALSE;
        if (xQueueSendFromISR(queue, &event, &higher_priority_task_woken) != pdPASS) {
            return false;
        }
        if (event.direction == UartTransferDirection::Transmit) {
            (void)startNextTransmit(true, &higher_priority_task_woken);
        } else {
            (void)startNextReceive(true, &higher_priority_task_woken);
        }
        portYIELD_FROM_ISR(higher_priority_task_woken);
        return true;
    }

    if (xQueueSend(queue, &event, 0U) != pdPASS) {
        return false;
    }

    if (event.direction == UartTransferDirection::Transmit) {
        (void)startNextTransmit(false);
    } else {
        (void)startNextReceive(false);
    }
    return true;
}

bool UartPort::takeNextTransmit(bool from_isr, BaseType_t* higher_priority_task_woken)
{
    if (tx_active_) {
        return false;
    }

    bool has_transfer = false;
    if (from_isr) {
        const UBaseType_t mask = portSET_INTERRUPT_MASK_FROM_ISR();
        has_transfer =
            xQueueReceiveFromISR(tx_queue_, &active_tx_, higher_priority_task_woken) == pdPASS;
        tx_active_ = has_transfer;
        portCLEAR_INTERRUPT_MASK_FROM_ISR(mask);
    } else {
        taskENTER_CRITICAL();
        has_transfer = xQueueReceive(tx_queue_, &active_tx_, 0U) == pdPASS;
        tx_active_ = has_transfer;
        taskEXIT_CRITICAL();
    }

    return has_transfer;
}

bool UartPort::takeNextReceive(bool from_isr, BaseType_t* higher_priority_task_woken)
{
    if (rx_active_) {
        return false;
    }

    bool has_transfer = false;
    if (from_isr) {
        const UBaseType_t mask = portSET_INTERRUPT_MASK_FROM_ISR();
        has_transfer =
            xQueueReceiveFromISR(rx_queue_, &active_rx_, higher_priority_task_woken) == pdPASS;
        rx_active_ = has_transfer;
        portCLEAR_INTERRUPT_MASK_FROM_ISR(mask);
    } else {
        taskENTER_CRITICAL();
        has_transfer = xQueueReceive(rx_queue_, &active_rx_, 0U) == pdPASS;
        rx_active_ = has_transfer;
        taskEXIT_CRITICAL();
    }

    return has_transfer;
}

bool UartPort::startNextTransmit(bool from_isr, BaseType_t* higher_priority_task_woken)
{
    if (!takeNextTransmit(from_isr, higher_priority_task_woken)) {
        return false;
    }

    if (!transmitEvent(active_tx_)) {
        completeTransmit(HAL_ERROR, from_isr, true);
        return false;
    }

    active_tx_.status = HAL_BUSY;
    if (active_tx_.before_cb != nullptr) {
        active_tx_.before_cb(active_tx_);
    }

    const HAL_StatusTypeDef status = startHalTransfer(active_tx_);
    if (status != HAL_OK) {
        completeTransmit(status, from_isr, true);
        return false;
    }

    return true;
}

bool UartPort::startNextReceive(bool from_isr, BaseType_t* higher_priority_task_woken)
{
    if (!takeNextReceive(from_isr, higher_priority_task_woken)) {
        return false;
    }

    if (!receiveEvent(active_rx_)) {
        completeReceive(HAL_ERROR, 0U, from_isr, true);
        return false;
    }

    active_rx_.status = HAL_BUSY;
    if (active_rx_.before_cb != nullptr) {
        active_rx_.before_cb(active_rx_);
    }

    const HAL_StatusTypeDef status = startHalTransfer(active_rx_);
    if (status != HAL_OK) {
        completeReceive(status, 0U, from_isr, true);
        return false;
    }

    return true;
}

void UartPort::clearTransmit(bool from_isr)
{
    if (from_isr) {
        const UBaseType_t mask = portSET_INTERRUPT_MASK_FROM_ISR();
        tx_active_ = false;
        portCLEAR_INTERRUPT_MASK_FROM_ISR(mask);
    } else {
        taskENTER_CRITICAL();
        tx_active_ = false;
        taskEXIT_CRITICAL();
    }
}

void UartPort::clearReceive(bool from_isr)
{
    if (from_isr) {
        const UBaseType_t mask = portSET_INTERRUPT_MASK_FROM_ISR();
        rx_active_ = false;
        portCLEAR_INTERRUPT_MASK_FROM_ISR(mask);
    } else {
        taskENTER_CRITICAL();
        rx_active_ = false;
        taskEXIT_CRITICAL();
    }
}

void UartPort::completeTransmit(HAL_StatusTypeDef status, bool from_isr, bool start_next)
{
    if (tx_active_) {
        active_tx_.status = status;
        active_tx_.error_code = HAL_UART_GetError(handle_);
        active_tx_.transferred_size = active_tx_.size;
        UartTransferEvent finished_event = active_tx_;
        clearTransmit(from_isr);
        if (finished_event.after_cb != nullptr) {
            finished_event.after_cb(finished_event);
        }
    }

    BaseType_t higher_priority_task_woken = pdFALSE;
    if (start_next) {
        (void)startNextTransmit(from_isr, &higher_priority_task_woken);
    }
    if (from_isr) {
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

void UartPort::completeReceive(HAL_StatusTypeDef status,
                               uint16_t transferred_size,
                               bool from_isr,
                               bool start_next)
{
    if (rx_active_) {
        active_rx_.status = status;
        active_rx_.error_code = HAL_UART_GetError(handle_);
        active_rx_.transferred_size = transferred_size;

        const bool keep_alive = status == HAL_OK && receiveKeepAlive(active_rx_);
        UartTransferEvent finished_event = active_rx_;
        if (!keep_alive) {
            clearReceive(from_isr);
        }
        if (finished_event.after_cb != nullptr) {
            finished_event.after_cb(finished_event);
        }
        if (keep_alive) {
            return;
        }
    }

    BaseType_t higher_priority_task_woken = pdFALSE;
    if (start_next) {
        (void)startNextReceive(from_isr, &higher_priority_task_woken);
    }
    if (from_isr) {
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

void UartPort::resetQueues()
{
    xQueueReset(tx_queue_);
    xQueueReset(rx_queue_);
}

bool UartPort::init()
{
    if (!valid()) {
        return false;
    }

    tx_active_ = false;
    rx_active_ = false;
    resetQueues();

#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
    bool registered = true;

    if (HAL_UART_RegisterCallback(handle_, HAL_UART_TX_HALFCOMPLETE_CB_ID,
                                  &UartPort::txHalfCpltCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_UART_RegisterCallback(handle_, HAL_UART_TX_COMPLETE_CB_ID,
                                  &UartPort::txCpltCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_UART_RegisterCallback(handle_, HAL_UART_RX_HALFCOMPLETE_CB_ID,
                                  &UartPort::rxHalfCpltCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_UART_RegisterCallback(handle_, HAL_UART_RX_COMPLETE_CB_ID,
                                  &UartPort::rxCpltCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_UART_RegisterCallback(handle_, HAL_UART_ERROR_CB_ID,
                                  &UartPort::errorCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_UART_RegisterCallback(handle_, HAL_UART_ABORT_COMPLETE_CB_ID,
                                  &UartPort::abortCpltCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_UART_RegisterCallback(handle_, HAL_UART_ABORT_TRANSMIT_COMPLETE_CB_ID,
                                  &UartPort::abortTransmitCpltCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_UART_RegisterCallback(handle_, HAL_UART_ABORT_RECEIVE_COMPLETE_CB_ID,
                                  &UartPort::abortReceiveCpltCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_UART_RegisterRxEventCallback(handle_, &UartPort::rxEventCallback) != HAL_OK) {
        registered = false;
    }

    if (registered) {
        instances()[handle_] = this;
    }
    return registered;
#else
    return false;
#endif
}

HAL_StatusTypeDef UartPort::submit(UartTransferEvent& event)
{
    if (!valid()) {
        return HAL_ERROR;
    }

    const bool is_tx = event.direction == UartTransferDirection::Transmit;
    const bool accepted = enqueueTransfer(event, is_tx ? tx_queue_ : rx_queue_);
    return accepted ? HAL_OK : HAL_BUSY;
}

HAL_StatusTypeDef UartPort::transmitIt(const void* data,
                                       uint16_t size,
                                       UartTransferEvent::Callback finished_cb,
                                       void* context)
{
    UartTransferEvent event = {};
    event.mode = UartTransferMode::Interrupt;
    event.direction = UartTransferDirection::Transmit;
    event.tx_data = data;
    event.size = size;
    event.context = context;
    event.after_cb = finished_cb;
    return submit(event);
}

HAL_StatusTypeDef UartPort::receiveIt(void* data,
                                      uint16_t size,
                                      UartTransferEvent::Callback finished_cb,
                                      void* context)
{
    UartTransferEvent event = {};
    event.mode = UartTransferMode::Interrupt;
    event.direction = UartTransferDirection::Receive;
    event.rx_data = data;
    event.size = size;
    event.context = context;
    event.after_cb = finished_cb;
    return submit(event);
}

HAL_StatusTypeDef UartPort::transmitDma(const void* data,
                                        uint16_t size,
                                        UartTransferEvent::Callback finished_cb,
                                        void* context)
{
    UartTransferEvent event = {};
    event.mode = UartTransferMode::Dma;
    event.direction = UartTransferDirection::Transmit;
    event.tx_data = data;
    event.size = size;
    event.context = context;
    event.after_cb = finished_cb;
    return submit(event);
}

HAL_StatusTypeDef UartPort::receiveDma(void* data,
                                       uint16_t size,
                                       UartTransferEvent::Callback finished_cb,
                                       void* context)
{
    UartTransferEvent event = {};
    event.mode = UartTransferMode::Dma;
    event.direction = UartTransferDirection::Receive;
    event.rx_data = data;
    event.size = size;
    event.context = context;
    event.after_cb = finished_cb;
    return submit(event);
}

HAL_StatusTypeDef UartPort::receiveToIdleDma(void* data,
                                             uint16_t size,
                                             UartTransferEvent::Callback finished_cb,
                                             void* context,
                                             bool disable_half_transfer_it)
{
    UartTransferEvent event = {};
    event.mode = UartTransferMode::Dma;
    event.direction = UartTransferDirection::ReceiveToIdle;
    event.hardware.disable_half_transfer_it = disable_half_transfer_it;
    event.rx_data = data;
    event.size = size;
    event.context = context;
    event.after_cb = finished_cb;
    return submit(event);
}

HAL_StatusTypeDef UartPort::stopDma()
{
    const HAL_StatusTypeDef status = HAL_UART_DMAStop(handle_);
    if (status == HAL_OK) {
        resetQueues();
        completeTransmit(HAL_ERROR, false, false);
        completeReceive(HAL_ERROR, 0U, false, false);
    }
    return status;
}

HAL_StatusTypeDef UartPort::abort()
{
    const HAL_StatusTypeDef status = HAL_UART_Abort(handle_);
    resetQueues();
    completeTransmit(status == HAL_OK ? HAL_ERROR : status, false, false);
    completeReceive(status == HAL_OK ? HAL_ERROR : status, 0U, false, false);
    return status;
}

HAL_StatusTypeDef UartPort::abortTransmit()
{
    const HAL_StatusTypeDef status = HAL_UART_AbortTransmit(handle_);
    completeTransmit(status == HAL_OK ? HAL_ERROR : status, false, true);
    return status;
}

HAL_StatusTypeDef UartPort::abortReceive()
{
    const HAL_StatusTypeDef status = HAL_UART_AbortReceive(handle_);
    completeReceive(status == HAL_OK ? HAL_ERROR : status, 0U, false, true);
    return status;
}

void UartPort::txHalfCpltCallback(UART_HandleTypeDef* huart)
{
    (void)huart;
}

void UartPort::txCpltCallback(UART_HandleTypeDef* huart)
{
    UartPort* port = find(huart);
    if (port != nullptr) {
        port->completeTransmit(HAL_OK, true, true);
    }
}

void UartPort::rxHalfCpltCallback(UART_HandleTypeDef* huart)
{
    (void)huart;
}

void UartPort::rxCpltCallback(UART_HandleTypeDef* huart)
{
    UartPort* port = find(huart);
    if (port != nullptr) {
        port->completeReceive(HAL_OK, port->active_rx_.size, true, true);
    }
}

void UartPort::errorCallback(UART_HandleTypeDef* huart)
{
    UartPort* port = find(huart);
    if (port != nullptr) {
        port->completeTransmit(HAL_ERROR, true, true);
        port->completeReceive(HAL_ERROR, 0U, true, true);
    }
}

void UartPort::abortCpltCallback(UART_HandleTypeDef* huart)
{
    UartPort* port = find(huart);
    if (port != nullptr) {
        port->completeTransmit(HAL_ERROR, true, true);
        port->completeReceive(HAL_ERROR, 0U, true, true);
    }
}

void UartPort::abortTransmitCpltCallback(UART_HandleTypeDef* huart)
{
    UartPort* port = find(huart);
    if (port != nullptr) {
        port->completeTransmit(HAL_ERROR, true, true);
    }
}

void UartPort::abortReceiveCpltCallback(UART_HandleTypeDef* huart)
{
    UartPort* port = find(huart);
    if (port != nullptr) {
        port->completeReceive(HAL_ERROR, 0U, true, true);
    }
}

void UartPort::rxEventCallback(UART_HandleTypeDef* huart, uint16_t size)
{
    UartPort* port = find(huart);
    if (port != nullptr) {
        port->completeReceive(HAL_OK, size, true, true);
    }
}

} // namespace bsp
