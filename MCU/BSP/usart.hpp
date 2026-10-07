#pragma once

#include "FreeRTOS.h"
#include "queue.h"
#include "stm32h7xx_hal.h"

#include <cstdint>
#include <unordered_map>

namespace bsp {

enum class UartTransferMode : uint8_t {
    Interrupt,
    Dma,
};

enum class UartTransferDirection : uint8_t {
    Transmit,
    Receive,
    ReceiveToIdle,
};

struct UartHardwareParams {
    bool disable_half_transfer_it = true;
    bool keep_alive = false;
};

struct UartTransferEvent {
    using Callback = void (*)(UartTransferEvent& event);

    UART_HandleTypeDef* handle = nullptr;
    UartTransferMode mode = UartTransferMode::Interrupt;
    UartTransferDirection direction = UartTransferDirection::Transmit;
    UartHardwareParams hardware = {};
    const void* tx_data = nullptr;
    void* rx_data = nullptr;
    uint16_t size = 0U;
    uint16_t transferred_size = 0U;
    void* context = nullptr;
    HAL_StatusTypeDef status = HAL_OK;
    uint32_t error_code = HAL_UART_ERROR_NONE;
    Callback before_cb = nullptr;
    Callback after_cb = nullptr;
};

class UartPort {
public:
    explicit UartPort(UART_HandleTypeDef& handle, uint32_t queue_length = 8U);
    ~UartPort();

    UartPort(const UartPort&) = delete;
    UartPort& operator=(const UartPort&) = delete;

    bool init();
    HAL_StatusTypeDef submit(UartTransferEvent& event);

    HAL_StatusTypeDef transmitIt(const void* data,
                                 uint16_t size,
                                 UartTransferEvent::Callback finished_cb = nullptr,
                                 void* context = nullptr);
    HAL_StatusTypeDef receiveIt(void* data,
                                uint16_t size,
                                UartTransferEvent::Callback finished_cb = nullptr,
                                void* context = nullptr);
    HAL_StatusTypeDef transmitDma(const void* data,
                                  uint16_t size,
                                  UartTransferEvent::Callback finished_cb = nullptr,
                                  void* context = nullptr);
    HAL_StatusTypeDef receiveDma(void* data,
                                 uint16_t size,
                                 UartTransferEvent::Callback finished_cb = nullptr,
                                 void* context = nullptr);
    HAL_StatusTypeDef receiveToIdleDma(void* data,
                                       uint16_t size,
                                       UartTransferEvent::Callback finished_cb = nullptr,
                                       void* context = nullptr,
                                       bool disable_half_transfer_it = true);

    HAL_StatusTypeDef stopDma();
    HAL_StatusTypeDef abort();
    HAL_StatusTypeDef abortTransmit();
    HAL_StatusTypeDef abortReceive();

    UART_HandleTypeDef* native() const { return handle_; }
    bool valid() const { return handle_ != nullptr && tx_queue_ != nullptr && rx_queue_ != nullptr; }

private:
    static void txHalfCpltCallback(UART_HandleTypeDef* huart);
    static void txCpltCallback(UART_HandleTypeDef* huart);
    static void rxHalfCpltCallback(UART_HandleTypeDef* huart);
    static void rxCpltCallback(UART_HandleTypeDef* huart);
    static void errorCallback(UART_HandleTypeDef* huart);
    static void abortCpltCallback(UART_HandleTypeDef* huart);
    static void abortTransmitCpltCallback(UART_HandleTypeDef* huart);
    static void abortReceiveCpltCallback(UART_HandleTypeDef* huart);
    static void rxEventCallback(UART_HandleTypeDef* huart, uint16_t size);

    static UartPort* find(UART_HandleTypeDef* huart);
    static std::unordered_map<UART_HandleTypeDef*, UartPort*>& instances();

    bool inIsr() const;
    bool transmitEvent(const UartTransferEvent& event) const;
    bool receiveEvent(const UartTransferEvent& event) const;
    bool enqueueTransfer(UartTransferEvent& event, QueueHandle_t queue);
    bool takeNextTransmit(bool from_isr, BaseType_t* higher_priority_task_woken);
    bool takeNextReceive(bool from_isr, BaseType_t* higher_priority_task_woken);
    bool startNextTransmit(bool from_isr, BaseType_t* higher_priority_task_woken = nullptr);
    bool startNextReceive(bool from_isr, BaseType_t* higher_priority_task_woken = nullptr);
    void completeTransmit(HAL_StatusTypeDef status, bool from_isr, bool start_next);
    void completeReceive(HAL_StatusTypeDef status,
                         uint16_t transferred_size,
                         bool from_isr,
                         bool start_next);
    void clearTransmit(bool from_isr);
    void clearReceive(bool from_isr);
    void resetQueues();
    HAL_StatusTypeDef startHalTransfer(UartTransferEvent& event) const;
    bool receiveKeepAlive(const UartTransferEvent& event) const;

    UART_HandleTypeDef* handle_ = nullptr;
    QueueHandle_t tx_queue_ = nullptr;
    QueueHandle_t rx_queue_ = nullptr;
    UartTransferEvent active_tx_ = {};
    UartTransferEvent active_rx_ = {};
    bool tx_active_ = false;
    bool rx_active_ = false;
};

} // namespace bsp
