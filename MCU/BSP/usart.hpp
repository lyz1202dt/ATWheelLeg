#pragma once

#include "FreeRTOS.h"
#include "queue.h"
#include "stm32h7xx_hal.h"

#include <cstdint>
#include <unordered_map>

namespace bsp {

enum class UartTransmitMode : uint8_t {
    Interrupt,
    Dma,
};

enum class UartReceiveMode : uint8_t {
    Interrupt,
    Dma,
    ToIdleDma,
};

struct UartTransmitEvent {
    using Callback = void (*)(UartTransmitEvent& event);

    UartTransmitMode mode = UartTransmitMode::Interrupt;
    const void* data = nullptr;
    uint16_t size = 0U;
    void* context = nullptr;
    HAL_StatusTypeDef status = HAL_OK;
    uint32_t error_code = HAL_UART_ERROR_NONE;
    Callback before_cb = nullptr;
    Callback after_cb = nullptr;
};

struct UartReceiveEvent {
    using Callback = void (*)(UartReceiveEvent& event);

    UartReceiveMode mode = UartReceiveMode::Interrupt;
    void* data = nullptr;
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
    HAL_StatusTypeDef transmit(UartTransmitEvent& event);
    HAL_StatusTypeDef receive(UartReceiveEvent& event);

    UART_HandleTypeDef* native() const { return handle_; }
    bool valid() const { return handle_ != nullptr && tx_queue_ != nullptr && rx_queue_ != nullptr; }

private:
    static void txCpltCallback(UART_HandleTypeDef* huart);
    static void rxCpltCallback(UART_HandleTypeDef* huart);
    static void errorCallback(UART_HandleTypeDef* huart);
    static void rxEventCallback(UART_HandleTypeDef* huart, uint16_t size);

    static UartPort* find(UART_HandleTypeDef* huart);
    static std::unordered_map<UART_HandleTypeDef*, UartPort*>& instances();

    bool startNextTransmit(bool from_isr, BaseType_t* higher_priority_task_woken = nullptr);
    bool startNextReceive(bool from_isr, BaseType_t* higher_priority_task_woken = nullptr);
    void completeTransmit(HAL_StatusTypeDef status, bool from_isr, bool start_next);
    void completeReceive(HAL_StatusTypeDef status,
                         uint16_t transferred_size,
                         bool from_isr,
                         bool start_next);

    UART_HandleTypeDef* handle_ = nullptr;
    QueueHandle_t tx_queue_ = nullptr;
    QueueHandle_t rx_queue_ = nullptr;
    UartTransmitEvent active_tx_ = {};
    UartReceiveEvent active_rx_ = {};
    bool tx_active_ = false;
    bool rx_active_ = false;
};

} // namespace bsp
