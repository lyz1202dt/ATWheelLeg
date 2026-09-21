#pragma once

#include "stm32h7xx_hal.h"
#include <functional>
#include <unordered_map>

namespace bsp {

class FdcanBus {
public:
    struct Frame {
        uint32_t id = 0U;
        uint32_t id_type = FDCAN_STANDARD_ID;
        uint32_t frame_type = FDCAN_DATA_FRAME;
        uint32_t data_length = FDCAN_DLC_BYTES_8;
        uint32_t error_state_indicator = FDCAN_ESI_ACTIVE;
        uint32_t bitrate_switch = FDCAN_BRS_OFF;
        uint32_t fd_format = FDCAN_CLASSIC_CAN;
        uint8_t data[64] = {};
    };

    explicit constexpr FdcanBus(FDCAN_HandleTypeDef &handle) : handle_(&handle) {}

    HAL_StatusTypeDef configFilter(const FDCAN_FilterTypeDef &filter) const;
    HAL_StatusTypeDef configGlobalFilter(uint32_t non_matching_std = FDCAN_REJECT,
                                         uint32_t non_matching_ext = FDCAN_REJECT,
                                         uint32_t reject_remote_std = FDCAN_FILTER_REMOTE,
                                         uint32_t reject_remote_ext = FDCAN_FILTER_REMOTE) const;
    HAL_StatusTypeDef configFifoWatermark(uint32_t fifo = FDCAN_CFG_RX_FIFO0,
                                          uint32_t watermark = 1U) const;
    HAL_StatusTypeDef configAcceptAllFilters(uint32_t std_filter_index = 0U,
                                             uint32_t ext_filter_index = 1U,
                                             uint32_t fifo = FDCAN_FILTER_TO_RXFIFO0) const;

    HAL_StatusTypeDef start(uint32_t active_its = FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                            uint32_t rx_fifo0_watermark = 1U) const;
    HAL_StatusTypeDef stop() const;

    HAL_StatusTypeDef activateNotification(uint32_t active_its,
                                           uint32_t buffer_indexes = 0U) const;
    HAL_StatusTypeDef deactivateNotification(uint32_t inactive_its) const;

    HAL_StatusTypeDef transmit(const Frame &frame,
                               std::function<void(void *)> cplt_cb = nullptr) const;
    HAL_StatusTypeDef transmit(uint32_t id, const uint8_t *data, uint8_t length = 8U,
                               uint32_t id_type = FDCAN_STANDARD_ID,
                               uint32_t fd_format = FDCAN_CLASSIC_CAN,
                               uint32_t bitrate_switch = FDCAN_BRS_OFF,
                               std::function<void(void *)> cplt_cb = nullptr) const;
    HAL_StatusTypeDef register_recv_cb(std::function<void(void *)> recv_cb,
                                       uint32_t rx_location = FDCAN_RX_FIFO0) const;
    HAL_StatusTypeDef receive(Frame &frame, uint32_t rx_location = FDCAN_RX_FIFO0) const;

    static uint32_t lengthToDlc(uint8_t length);
    static uint8_t dlcToLength(uint32_t dlc);

private:
    static FDCAN_TxHeaderTypeDef makeTxHeader(const Frame &frame);
    HAL_StatusTypeDef registerCallbacks() const;

    static void clockCalibrationCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t clk_calibration_its);
    static void txEventFifoCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t tx_event_fifo_its);
    static void rxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo0_its);
    static void rxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo1_its);
    static void txFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan);
    static void txBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t buffer_indexes);
    static void txBufferAbortCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t buffer_indexes);
    static void rxBufferNewMessageCallback(FDCAN_HandleTypeDef *hfdcan);
    static void highPriorityMessageCallback(FDCAN_HandleTypeDef *hfdcan);
    static void timestampWraparoundCallback(FDCAN_HandleTypeDef *hfdcan);
    static void timeoutOccurredCallback(FDCAN_HandleTypeDef *hfdcan);
    static void errorCallback(FDCAN_HandleTypeDef *hfdcan);
    static void errorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t error_status_its);
    static void ttScheduleSyncCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t tt_sched_sync_its);
    static void ttTimeMarkCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t tt_time_mark_its);
    static void ttStopWatchCallback(FDCAN_HandleTypeDef *hfdcan,
                                     uint32_t sw_time,
                                     uint32_t sw_cycle_count);
    static void ttGlobalTimeCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t tt_global_time_its);

    // HAL 句柄和类接口绑定，静态回调中通过这个区分不同的设备。
    static std::unordered_map<FDCAN_HandleTypeDef *, FdcanBus *> handle_map;
    FDCAN_HandleTypeDef *handle_ = nullptr;
};

} // namespace bsp
