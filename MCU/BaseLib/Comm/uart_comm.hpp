#pragma once

#include "comm.hpp"
#include "pack.h"
#include "usart.hpp"

#include <cstddef>
#include <cstdint>

namespace comm {

class UartComm : public Comm {
public:
    static constexpr std::size_t kMaxPayloadSize = 120U;
    static constexpr std::size_t kFrameHeadSize = 1U + 1U + sizeof(uint32_t);
    static constexpr std::size_t kFrameCrcSize = sizeof(uint16_t);
    static constexpr std::size_t kFrameMaxSize = kFrameHeadSize + kMaxPayloadSize + kFrameCrcSize;
    static constexpr std::size_t kCommandMaxSize = sizeof(uint32_t) + sizeof(uint32_t) + kMaxPayloadSize;
    static constexpr std::size_t kRxDmaBufferSize = 256U;
    static constexpr std::size_t kRxRingBufferSize = 512U;

    explicit UartComm(bsp::UartPort& port);

    UartComm(const UartComm&) = delete;
    UartComm& operator=(const UartComm&) = delete;

    bool init() override;
    bool set_dma_address(uint8_t* rx_buffer, std::size_t rx_size,
                         uint8_t* tx_buffer, std::size_t tx_size);
    void register_recv_callback(ReceiveCallback callback) override;
    bool send_pack(void* data, int size) override;

private:
    static constexpr uint8_t kFrameHeader = 0x5AU;
    static constexpr std::size_t kHeaderOffset = 0U;
    static constexpr std::size_t kLengthOffset = 1U;
    static constexpr std::size_t kCommandOffset = 2U;
    static constexpr std::size_t kPayloadOffset = kFrameHeadSize;

    static void uartEventCallback(bsp::UartTransferEvent& event);
    static uint16_t crc16Modbus(const uint8_t* data, std::size_t size);
    static uint16_t readU16Le(const uint8_t* data);
    static uint32_t readU32Le(const uint8_t* data);
    static void writeU16Le(uint8_t* data, uint16_t value);
    static void writeU32Le(uint8_t* data, uint32_t value);

    bool configureReceiveDma() const;
    bool startReceive();
    void onTransmitComplete(HAL_StatusTypeDef status);
    void onReceiveEvent(uint16_t position, HAL_StatusTypeDef status);
    void pushDmaRange(std::size_t begin, std::size_t end);
    void pushRxByte(uint8_t value);
    uint8_t peekRxByte(std::size_t offset) const;
    void dropRxBytes(std::size_t count);
    void parseRxRing();
    bool tryParseFrame();

    bsp::UartPort* port_ = nullptr;
    ReceiveCallback recv_callback_;
    uint8_t rx_dma_buffer_storage_[kRxDmaBufferSize] = {};
    uint8_t* rx_dma_buffer_ = rx_dma_buffer_storage_;
    std::size_t rx_dma_buffer_size_ = kRxDmaBufferSize;
    uint8_t rx_ring_buffer_[kRxRingBufferSize] = {};
    std::size_t rx_dma_last_pos_ = 0U;
    std::size_t rx_ring_head_ = 0U;
    std::size_t rx_ring_tail_ = 0U;
    std::size_t rx_ring_size_ = 0U;
    uint8_t send_buffer_storage_[kFrameMaxSize] = {};
    uint8_t* send_buffer_ = send_buffer_storage_;
    std::size_t send_buffer_size_ = kFrameMaxSize;
    alignas(uint32_t) uint8_t command_buffer_[kCommandMaxSize] = {};
    volatile bool transmit_busy_ = false;
};

} // namespace comm
