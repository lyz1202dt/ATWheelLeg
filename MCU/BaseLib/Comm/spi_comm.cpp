#include "spi_comm.hpp"

#include <cstring>
#include <utility>

namespace comm {

SpiComm::SpiComm(bsp::SpiBus& bus, bsp::SpiTransferMode mode)
    : bus_(&bus),
      mode_(mode)
{
}

bool SpiComm::init()
{
    return bus_ != nullptr && bus_->valid();
}

void SpiComm::register_recv_callback(ReceiveCallback callback)
{
    recv_callback_ = std::move(callback);
}

void SpiComm::set_hardware(const bsp::SpiHardwareParams& hardware)
{
    hardware_ = hardware;
}

void SpiComm::set_event_callbacks(bsp::SpiTransferEvent::Callback before_cb,
                                  bsp::SpiTransferEvent::Callback after_cb,
                                  void* context)
{
    user_before_cb_ = before_cb;
    user_after_cb_ = after_cb;
    user_context_ = context;
}

bool SpiComm::send_pack(void* data, int size)
{
    return startTransfer(data, size, false);
}

bool SpiComm::transfer_pack(void* data, int size)
{
    return startTransfer(data, size, true);
}

bool SpiComm::startTransfer(void* data, int size, bool receive)
{
    if (data == nullptr || size <= 0 || size > static_cast<int>(kMaxPacketSize)) {
        return false;
    }

    bool accepted = false;
    __disable_irq();
    if (!busy_) {
        busy_ = true;
        accepted = true;
    }
    __enable_irq();

    if (!accepted) {
        return false;
    }

    std::memcpy(tx_buffer_, data, static_cast<std::size_t>(size));
    if (receive) {
        std::memset(rx_buffer_, 0, static_cast<std::size_t>(size));
    }
    receive_on_complete_ = receive;

    bsp::SpiTransferEvent event = {};
    event.mode = mode_;
    event.hardware = hardware_;
    event.tx_data = tx_buffer_;
    event.rx_data = receive ? rx_buffer_ : nullptr;
    event.size = static_cast<uint16_t>(size);
    event.context = this;
    event.before_cb = user_before_cb_ != nullptr ? &SpiComm::spiBeforeCallback : nullptr;
    event.after_cb = &SpiComm::spiEventCallback;

    const bool queued = bus_->submit(event);
    if (!queued) {
        busy_ = false;
        receive_on_complete_ = false;
        return false;
    }

    return true;
}

void SpiComm::spiBeforeCallback(bsp::SpiTransferEvent& event)
{
    auto* comm = static_cast<SpiComm*>(event.context);
    bsp::SpiTransferEvent user_event = event;
    user_event.context = comm->user_context_;
    comm->user_before_cb_(user_event);
}

void SpiComm::spiEventCallback(bsp::SpiTransferEvent& event)
{
    auto* comm = static_cast<SpiComm*>(event.context);
    comm->onTransferComplete(event);
}

void SpiComm::onTransferComplete(bsp::SpiTransferEvent& event)
{
    const bool should_dispatch_rx =
        receive_on_complete_ && event.status == bsp::SpiTransferStatus::Ok && recv_callback_;

    if (user_after_cb_ != nullptr) {
        bsp::SpiTransferEvent user_event = event;
        user_event.context = user_context_;
        user_after_cb_(user_event);
    }

    if (should_dispatch_rx) {
        recv_callback_(rx_buffer_, event.size);
    }

    __disable_irq();
    busy_ = false;
    receive_on_complete_ = false;
    __enable_irq();
}

} // namespace comm
