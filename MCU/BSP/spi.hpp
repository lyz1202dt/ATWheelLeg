#pragma once

#include "FreeRTOS.h"
#include "queue.h"
#include "stm32h7xx_hal.h"

#include <cstdint>
#include <unordered_map>

namespace bsp {

enum class SpiTransferMode : uint8_t {
    Interrupt,
    Dma,
};

enum class SpiTransferType : uint8_t {
    Transmit,
    Receive,
    TransmitReceive,
};

struct SpiHardwareParams {
    uint32_t data_size = 0U;
    uint32_t clk_polarity = 0U;
    uint32_t clk_phase = 0U;
    uint32_t first_bit = 0U;
};

struct SpiTransferEvent {
    using Callback = void (*)(SpiTransferEvent& event);

    SPI_HandleTypeDef* handle = nullptr;
    SpiTransferMode mode = SpiTransferMode::Interrupt;
    SpiTransferType type = SpiTransferType::TransmitReceive;
    SpiHardwareParams hardware = {};
    const void* tx_data = nullptr;
    void* rx_data = nullptr;
    uint16_t size = 0U;
    void* context = nullptr;
    HAL_StatusTypeDef status = HAL_OK;
    uint32_t error_code = HAL_SPI_ERROR_NONE;
    Callback before_cb = nullptr;
    Callback after_cb = nullptr;
};

using SPITransfer = SpiTransferEvent;

class SpiBus {
public:
    explicit SpiBus(SPI_HandleTypeDef& handle, uint32_t queue_length = 8U);
    ~SpiBus();

    SpiBus(const SpiBus&) = delete;
    SpiBus& operator=(const SpiBus&) = delete;

    bool init();
    bool submit(SpiTransferEvent& event);
    bool transferIt(SpiTransferEvent& event);
    bool transferDma(SpiTransferEvent& event);
    bool abort();

    SPI_HandleTypeDef* native() const { return handle_; }
    bool valid() const { return handle_ != nullptr && queue_ != nullptr; }

private:
    static void txCpltCallback(SPI_HandleTypeDef* hspi);
    static void rxCpltCallback(SPI_HandleTypeDef* hspi);
    static void txRxCpltCallback(SPI_HandleTypeDef* hspi);
    static void txHalfCpltCallback(SPI_HandleTypeDef* hspi);
    static void rxHalfCpltCallback(SPI_HandleTypeDef* hspi);
    static void txRxHalfCpltCallback(SPI_HandleTypeDef* hspi);
    static void errorCallback(SPI_HandleTypeDef* hspi);
    static void abortCpltCallback(SPI_HandleTypeDef* hspi);
    static void suspendCallback(SPI_HandleTypeDef* hspi);

    static SpiBus* find(SPI_HandleTypeDef* hspi);
    static std::unordered_map<SPI_HandleTypeDef*, SpiBus*>& instances();

    bool inIsr() const;
    bool transferValid(const SpiTransferEvent& event) const;
    bool enqueueTransfer(SpiTransferEvent& event);
    bool takeNextTransfer(bool from_isr, BaseType_t* higher_priority_task_woken);
    bool startNextTransfer(bool from_isr, BaseType_t* higher_priority_task_woken = nullptr);
    void completeTransfer(HAL_StatusTypeDef status, bool from_isr, bool start_next);
    void clearActiveTransfer(bool from_isr);
    HAL_StatusTypeDef startHalTransfer(SpiTransferEvent& event) const;
    HAL_StatusTypeDef applyHardwareParams(const SpiTransferEvent& event) const;

    SPI_HandleTypeDef* handle_ = nullptr;
    QueueHandle_t queue_ = nullptr;
    SpiTransferEvent active_event_ = {};
    bool active_ = false;
};

} // namespace bsp
