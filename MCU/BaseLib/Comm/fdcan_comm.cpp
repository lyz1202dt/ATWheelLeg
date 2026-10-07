#include "fdcan_comm.hpp"

#include <cstring>
#include <utility>

namespace comm {

FdcanComm::FdcanComm(bsp::FdcanBus& bus,
                     uint32_t tx_id,
                     uint32_t rx_id,
                     uint32_t id_type,
                     uint32_t fd_format,
                     uint32_t bitrate_switch)
    : bus_(&bus),
      tx_id_(tx_id),
      rx_id_(rx_id),
      id_type_(id_type),
      fd_format_(fd_format),
      bitrate_switch_(bitrate_switch)
{
}

bool FdcanComm::init()
{
    if (registered_) {
        return true;
    }

    const HAL_StatusTypeDef status =
        bus_->register_recv_cb([this](const bsp::FdcanBus::Frame& frame) {
            onFrame(frame);
        });
    registered_ = status == HAL_OK;
    return registered_;
}

void FdcanComm::register_recv_callback(ReceiveCallback callback)
{
    recv_callback_ = std::move(callback);
}

bool FdcanComm::send_pack(void* data, int size)
{
    if (data == nullptr || size <= 0 || size > maxFrameSize()) {
        return false;
    }

    return bus_->transmit(tx_id_,
                          static_cast<const uint8_t*>(data),
                          static_cast<uint8_t>(size),
                          id_type_,
                          fd_format_,
                          bitrate_switch_) == HAL_OK;
}

uint8_t FdcanComm::maxFrameSize() const
{
    return fd_format_ == FDCAN_CLASSIC_CAN ? kClassicMaxSize : kFdMaxSize;
}

void FdcanComm::onFrame(const bsp::FdcanBus::Frame& frame)
{
    if (frame.id != rx_id_) {
        return;
    }

    const uint8_t length = bsp::FdcanBus::dlcToLength(frame.data_length);
    if (length == 0U || length > sizeof(rx_buffer_)) {
        return;
    }

    std::memcpy(rx_buffer_, frame.data, length);
    if (recv_callback_) {
        recv_callback_(rx_buffer_, length);
    }
}

} // namespace comm
