#include "uart_comm.hpp"

#include <cstring>
#include <utility>

namespace comm {

UartComm::UartComm(bsp::UartPort& port)
    : port_(&port)
{
}

void UartComm::register_recv_callback(ReceiveCallback callback)
{
    recv_callback_ = std::move(callback);
}

bool UartComm::set_dma_address(uint8_t* rx_buffer, std::size_t rx_size,
                               uint8_t* tx_buffer, std::size_t tx_size)
{
    if (rx_buffer == nullptr || rx_size == 0U || rx_size > 0xFFFFU ||
        tx_buffer == nullptr || tx_size < kFrameMaxSize || transmit_busy_) {
        return false;
    }

    rx_dma_buffer_ = rx_buffer;
    rx_dma_buffer_size_ = rx_size;
    send_buffer_ = tx_buffer;
    send_buffer_size_ = tx_size;
    return true;
}

bool UartComm::init()
{
    if (port_ == nullptr || !port_->valid() || port_->native()->hdmarx == nullptr) {
        return false;
    }

    DMA_HandleTypeDef* rx_dma = port_->native()->hdmarx;
    if (rx_dma->Init.Mode != DMA_NORMAL) {
        rx_dma->Init.Mode = DMA_NORMAL;
        if (HAL_DMA_Init(rx_dma) != HAL_OK) {
            return false;
        }
    }

    rx_ring_head_ = 0U;
    rx_ring_tail_ = 0U;
    rx_ring_size_ = 0U;
    transmit_busy_ = false;

    return startReceive();
}

bool UartComm::send_pack(void* data, int size)
{
    if (port_ == nullptr || data == nullptr || size < static_cast<int>(sizeof(Command))) {
        return false;
    }

    const auto* command = static_cast<const Command*>(data);
    const uint32_t payload_size = command->size;
    if (payload_size > kMaxPayloadSize ||
        size != static_cast<int>(sizeof(Command) + payload_size)) {
        return false;
    }

    bool accepted = false;
    __disable_irq();
    if (!transmit_busy_) {
        transmit_busy_ = true;
        accepted = true;
    }
    __enable_irq();

    if (!accepted) {
        return false;
    }

    const uint16_t frame_size =
        static_cast<uint16_t>(kFrameHeadSize + payload_size + kFrameCrcSize);
    if (frame_size > send_buffer_size_) {
        onTransmitComplete();
        return false;
    }

    send_buffer_[kHeaderOffset] = kFrameHeader;
    send_buffer_[kLengthOffset] = static_cast<uint8_t>(payload_size);
    writeU32Le(&send_buffer_[kCommandOffset], command->cmd);
    if (payload_size > 0U) {
        std::memcpy(&send_buffer_[kPayloadOffset], command->data, payload_size);
    }

    const uint16_t crc = crc16Modbus(send_buffer_, frame_size - kFrameCrcSize);
    writeU16Le(&send_buffer_[frame_size - kFrameCrcSize], crc);

    bsp::UartTransmitEvent event = {};
    event.mode = bsp::UartTransmitMode::Dma;
    event.data = send_buffer_;
    event.size = frame_size;
    event.context = this;
    event.after_cb = &UartComm::uartTransmitCallback;

    const HAL_StatusTypeDef status = port_->transmit(event);
    if (status != HAL_OK) {
        onTransmitComplete();
        return false;
    }

    return true;
}

void UartComm::uartTransmitCallback(bsp::UartTransmitEvent& event)
{
    auto* comm = static_cast<UartComm*>(event.context);
    if (comm != nullptr) {
        comm->onTransmitComplete();
    }
}

void UartComm::uartReceiveCallback(bsp::UartReceiveEvent& event)
{
    auto* comm = static_cast<UartComm*>(event.context);
    if (comm != nullptr) {
        comm->onReceiveEvent(event.transferred_size, event.status);
    }
}

bool UartComm::startReceive()
{
    bsp::UartReceiveEvent event = {};
    event.mode = bsp::UartReceiveMode::ToIdleDma;
    event.data = rx_dma_buffer_;
    event.size = static_cast<uint16_t>(rx_dma_buffer_size_);
    event.context = this;
    event.after_cb = &UartComm::uartReceiveCallback;
    return port_->receive(event) == HAL_OK;
}

void UartComm::onTransmitComplete()
{
    __disable_irq();
    transmit_busy_ = false;
    __enable_irq();
}

void UartComm::onReceiveEvent(uint16_t position, HAL_StatusTypeDef status)
{
    if (status != HAL_OK) {
        (void)startReceive();
        return;
    }

    std::size_t current_pos = position;
    if (current_pos > rx_dma_buffer_size_) {
        current_pos = rx_dma_buffer_size_;
    }

    for (std::size_t i = 0U; i < current_pos; ++i) {
        pushRxByte(rx_dma_buffer_[i]);
    }

    (void)startReceive();
    parseRxRing();
}

void UartComm::pushRxByte(uint8_t value)
{
    rx_ring_buffer_[rx_ring_head_] = value;
    rx_ring_head_ = (rx_ring_head_ + 1U) % kRxRingBufferSize;

    if (rx_ring_size_ == kRxRingBufferSize) {
        rx_ring_tail_ = (rx_ring_tail_ + 1U) % kRxRingBufferSize;
    } else {
        ++rx_ring_size_;
    }
}

uint8_t UartComm::peekRxByte(std::size_t offset) const
{
    return rx_ring_buffer_[(rx_ring_tail_ + offset) % kRxRingBufferSize];
}

void UartComm::dropRxBytes(std::size_t count)
{
    if (count > rx_ring_size_) {
        count = rx_ring_size_;
    }

    rx_ring_tail_ = (rx_ring_tail_ + count) % kRxRingBufferSize;
    rx_ring_size_ -= count;
}

void UartComm::parseRxRing()
{
    while (tryParseFrame()) {
    }
}

bool UartComm::tryParseFrame()
{
    constexpr std::size_t min_frame_size = kFrameHeadSize + kFrameCrcSize;

    while (rx_ring_size_ >= min_frame_size && peekRxByte(0U) != kFrameHeader) {
        dropRxBytes(1U);
    }

    if (rx_ring_size_ < min_frame_size) {
        return false;
    }

    const uint8_t payload_size = peekRxByte(kLengthOffset);
    if (payload_size > kMaxPayloadSize) {
        dropRxBytes(1U);
        return true;
    }

    const std::size_t frame_size = kFrameHeadSize + payload_size + kFrameCrcSize;
    if (rx_ring_size_ < frame_size) {
        return false;
    }

    uint8_t frame[kFrameMaxSize] = {};
    for (std::size_t i = 0U; i < frame_size; ++i) {
        frame[i] = peekRxByte(i);
    }

    const uint16_t received_crc = readU16Le(&frame[frame_size - kFrameCrcSize]);
    const uint16_t expected_crc = crc16Modbus(frame, frame_size - kFrameCrcSize);
    if (received_crc != expected_crc) {
        dropRxBytes(1U);
        return true;
    }

    auto* command = reinterpret_cast<Command*>(command_buffer_);
    command->cmd = readU32Le(&frame[kCommandOffset]);
    command->size = payload_size;
    if (payload_size > 0U) {
        std::memcpy(command->data, &frame[kPayloadOffset], payload_size);
    }

    dropRxBytes(frame_size);

    if (recv_callback_) {
        recv_callback_(command_buffer_, static_cast<int>(sizeof(Command) + payload_size));
    }

    return rx_ring_size_ >= min_frame_size;
}

uint16_t UartComm::crc16Modbus(const uint8_t* data, std::size_t size)
{
    uint16_t crc = 0xFFFFU;
    for (std::size_t i = 0U; i < size; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            if ((crc & 0x0001U) != 0U) {
                crc = static_cast<uint16_t>((crc >> 1U) ^ 0xA001U);
            } else {
                crc >>= 1U;
            }
        }
    }
    return crc;
}

uint16_t UartComm::readU16Le(const uint8_t* data)
{
    return static_cast<uint16_t>(data[0]) |
           static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8U);
}

uint32_t UartComm::readU32Le(const uint8_t* data)
{
    return static_cast<uint32_t>(data[0]) |
           (static_cast<uint32_t>(data[1]) << 8U) |
           (static_cast<uint32_t>(data[2]) << 16U) |
           (static_cast<uint32_t>(data[3]) << 24U);
}

void UartComm::writeU16Le(uint8_t* data, uint16_t value)
{
    data[0] = static_cast<uint8_t>(value & 0x00FFU);
    data[1] = static_cast<uint8_t>((value >> 8U) & 0x00FFU);
}

void UartComm::writeU32Le(uint8_t* data, uint32_t value)
{
    data[0] = static_cast<uint8_t>(value & 0x000000FFUL);
    data[1] = static_cast<uint8_t>((value >> 8U) & 0x000000FFUL);
    data[2] = static_cast<uint8_t>((value >> 16U) & 0x000000FFUL);
    data[3] = static_cast<uint8_t>((value >> 24U) & 0x000000FFUL);
}

} // namespace comm
