#include "usart.hpp"

#include "task.h"

namespace bsp {

UartPort::UartPort(UART_HandleTypeDef& handle, uint32_t queue_length)
    : handle_(&handle)
{
    if (queue_length != 0U) {
        tx_queue_ = xQueueCreate(static_cast<UBaseType_t>(queue_length),
                                 sizeof(UartTransmitEvent));
        rx_queue_ = xQueueCreate(static_cast<UBaseType_t>(queue_length),
                                 sizeof(UartReceiveEvent));
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

bool UartPort::startNextTransmit(bool from_isr, BaseType_t* higher_priority_task_woken)
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

    if (!has_transfer) {
        return false;
    }

    if (active_tx_.data == nullptr || active_tx_.size == 0U) {
        completeTransmit(HAL_ERROR, from_isr, true);
        return false;
    }

    active_tx_.status = HAL_BUSY;
    if (active_tx_.before_cb != nullptr) {
        active_tx_.before_cb(active_tx_);
    }

    const auto* data = static_cast<const uint8_t*>(active_tx_.data);
    const HAL_StatusTypeDef status =
        active_tx_.mode == UartTransmitMode::Dma
            ? HAL_UART_Transmit_DMA(handle_, data, active_tx_.size)
            : HAL_UART_Transmit_IT(handle_, data, active_tx_.size);
    if (status == HAL_OK && active_tx_.mode == UartTransmitMode::Dma &&
        handle_->hdmatx != nullptr) {
        __HAL_DMA_DISABLE_IT(handle_->hdmatx, DMA_IT_HT);
    }
    if (status != HAL_OK) {
        completeTransmit(status, from_isr, true);
        return false;
    }

    return true;
}

bool UartPort::startNextReceive(bool from_isr, BaseType_t* higher_priority_task_woken)
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

    if (!has_transfer) {
        return false;
    }

    if (active_rx_.data == nullptr || active_rx_.size == 0U) {
        completeReceive(HAL_ERROR, 0U, from_isr, true);
        return false;
    }

    active_rx_.status = HAL_BUSY;
    if (active_rx_.before_cb != nullptr) {
        active_rx_.before_cb(active_rx_);
    }

    auto* data = static_cast<uint8_t*>(active_rx_.data);
    const bool use_dma = active_rx_.mode != UartReceiveMode::Interrupt;
    HAL_StatusTypeDef status = HAL_ERROR;
    if (active_rx_.mode == UartReceiveMode::ToIdleDma) {
        status = HAL_UARTEx_ReceiveToIdle_DMA(handle_, data, active_rx_.size);
    } else if (active_rx_.mode == UartReceiveMode::Dma) {
        status = HAL_UART_Receive_DMA(handle_, data, active_rx_.size);
    } else {
        status = HAL_UART_Receive_IT(handle_, data, active_rx_.size);
    }
    if (status == HAL_OK && use_dma && handle_->hdmarx != nullptr) {
        __HAL_DMA_DISABLE_IT(handle_->hdmarx, DMA_IT_HT);
    }
    if (status != HAL_OK) {
        completeReceive(status, 0U, from_isr, true);
        return false;
    }

    return true;
}

void UartPort::completeTransmit(HAL_StatusTypeDef status, bool from_isr, bool start_next)
{
    if (tx_active_) {
        active_tx_.status = status;
        active_tx_.error_code = HAL_UART_GetError(handle_);
        UartTransmitEvent finished_event = active_tx_;
        if (from_isr) {
            const UBaseType_t mask = portSET_INTERRUPT_MASK_FROM_ISR();
            tx_active_ = false;
            portCLEAR_INTERRUPT_MASK_FROM_ISR(mask);
        } else {
            taskENTER_CRITICAL();
            tx_active_ = false;
            taskEXIT_CRITICAL();
        }
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
        UartReceiveEvent finished_event = active_rx_;
        if (from_isr) {
            const UBaseType_t mask = portSET_INTERRUPT_MASK_FROM_ISR();
            rx_active_ = false;
            portCLEAR_INTERRUPT_MASK_FROM_ISR(mask);
        } else {
            taskENTER_CRITICAL();
            rx_active_ = false;
            taskEXIT_CRITICAL();
        }
        if (finished_event.after_cb != nullptr) {
            finished_event.after_cb(finished_event);
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

bool UartPort::init()
{
    if (!valid()) {
        return false;
    }

    tx_active_ = false;
    rx_active_ = false;
    xQueueReset(tx_queue_);
    xQueueReset(rx_queue_);

#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
    bool registered = true;

    if (HAL_UART_RegisterCallback(handle_, HAL_UART_TX_COMPLETE_CB_ID,
                                  &UartPort::txCpltCallback) != HAL_OK) {
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

HAL_StatusTypeDef UartPort::transmit(UartTransmitEvent& event)
{
    if (!valid()) {
        return HAL_ERROR;
    }

    event.status = HAL_OK;
    event.error_code = HAL_UART_ERROR_NONE;

    if (__get_IPSR() != 0U) {
        BaseType_t higher_priority_task_woken = pdFALSE;
        if (xQueueSendFromISR(tx_queue_, &event, &higher_priority_task_woken) != pdPASS) {
            return HAL_BUSY;
        }
        (void)startNextTransmit(true, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
        return HAL_OK;
    }

    if (xQueueSend(tx_queue_, &event, 0U) != pdPASS) {
        return HAL_BUSY;
    }

    (void)startNextTransmit(false);
    return HAL_OK;
}

HAL_StatusTypeDef UartPort::receive(UartReceiveEvent& event)
{
    if (!valid()) {
        return HAL_ERROR;
    }

    event.status = HAL_OK;
    event.error_code = HAL_UART_ERROR_NONE;
    event.transferred_size = 0U;

    if (__get_IPSR() != 0U) {
        BaseType_t higher_priority_task_woken = pdFALSE;
        if (xQueueSendFromISR(rx_queue_, &event, &higher_priority_task_woken) != pdPASS) {
            return HAL_BUSY;
        }
        (void)startNextReceive(true, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
        return HAL_OK;
    }

    if (xQueueSend(rx_queue_, &event, 0U) != pdPASS) {
        return HAL_BUSY;
    }

    (void)startNextReceive(false);
    return HAL_OK;
}

void UartPort::txCpltCallback(UART_HandleTypeDef* huart)
{
    UartPort* port = find(huart);
    if (port != nullptr) {
        port->completeTransmit(HAL_OK, true, true);
    }
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

void UartPort::rxEventCallback(UART_HandleTypeDef* huart, uint16_t size)
{
    UartPort* port = find(huart);
    if (port != nullptr) {
        port->completeReceive(HAL_OK, size, true, true);
    }
}

} // namespace bsp
