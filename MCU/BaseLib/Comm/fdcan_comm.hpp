#pragma once

#include "comm.hpp"
#include "fdcan.hpp"

#include <cstdint>

namespace comm {

class FdcanComm : public Comm {
public:
    static constexpr uint8_t kClassicMaxSize = 8U;
    static constexpr uint8_t kFdMaxSize = 64U;

    FdcanComm(bsp::FdcanBus& bus,
              uint32_t tx_id,
              uint32_t rx_id,
              uint32_t id_type = FDCAN_STANDARD_ID,
              uint32_t fd_format = FDCAN_FD_CAN,
              uint32_t bitrate_switch = FDCAN_BRS_OFF);

    bool init() override;
    void register_recv_callback(ReceiveCallback callback) override;
    bool send_pack(void* data, int size) override;

private:
    uint8_t maxFrameSize() const;
    void onFrame(const bsp::FdcanBus::Frame& frame);

    bsp::FdcanBus* bus_ = nullptr;
    uint32_t tx_id_ = 0U;
    uint32_t rx_id_ = 0U;
    uint32_t id_type_ = FDCAN_STANDARD_ID;
    uint32_t fd_format_ = FDCAN_FD_CAN;
    uint32_t bitrate_switch_ = FDCAN_BRS_OFF;
    ReceiveCallback recv_callback_;
    uint8_t rx_buffer_[kFdMaxSize] = {};
    bool registered_ = false;
};

} // namespace comm
