#pragma once

#include "comm.hpp"
#include "spi.hpp"

#include <cstddef>
#include <cstdint>

namespace comm {

class SpiComm : public Comm {
public:
    static constexpr std::size_t kMaxPacketSize = 128U;

    explicit SpiComm(bsp::SpiBus& bus,
                     bsp::SpiTransferMode mode = bsp::SpiTransferMode::Interrupt);

    bool init() override;
    void register_recv_callback(ReceiveCallback callback) override;
    bool send_pack(void* data, int size) override;
    bool transfer_pack(void* data, int size);
    bool busy() const { return busy_; }

    void set_hardware(const bsp::SpiHardwareParams& hardware);
    void set_event_callbacks(bsp::SpiTransferEvent::Callback before_cb,
                             bsp::SpiTransferEvent::Callback after_cb,
                             void* context);

private:
    static void spiBeforeCallback(bsp::SpiTransferEvent& event);
    static void spiEventCallback(bsp::SpiTransferEvent& event);
    bool startTransfer(void* data, int size, bool receive);
    void onTransferComplete(bsp::SpiTransferEvent& event);

    bsp::SpiBus* bus_ = nullptr;
    bsp::SpiTransferMode mode_ = bsp::SpiTransferMode::Interrupt;
    bsp::SpiHardwareParams hardware_ = {};
    bsp::SpiTransferEvent::Callback user_before_cb_ = nullptr;
    bsp::SpiTransferEvent::Callback user_after_cb_ = nullptr;
    void* user_context_ = nullptr;
    ReceiveCallback recv_callback_;
    uint8_t tx_buffer_[kMaxPacketSize] = {};
    uint8_t rx_buffer_[kMaxPacketSize] = {};
    volatile bool busy_ = false;
    bool receive_on_complete_ = false;
};

} // namespace comm
