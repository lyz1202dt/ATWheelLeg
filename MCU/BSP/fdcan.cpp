#include "fdcan.hpp"

#include <utility>
#include <vector>

namespace bsp {

namespace {

struct TxTransfer {
    FdcanBus::Frame frame = {};
    std::function<void(void *)> finished_callback;
};

struct CallbackState {
    std::unordered_map<uint32_t, std::vector<std::function<void(const FdcanBus::Frame &)>>> receive_callbacks;

    bool pending_tx = false;
    TxTransfer pending_transfer = {};
    std::unordered_map<uint32_t, TxTransfer> tx_transfers;
};

std::unordered_map<FDCAN_HandleTypeDef *, CallbackState> callback_states;

CallbackState *findCallbackState(FDCAN_HandleTypeDef *hfdcan)
{
    const auto it = callback_states.find(hfdcan);
    if (it == callback_states.end()) {
        return nullptr;
    }
    return &it->second;
}

void clearPendingTransfer(CallbackState &state)
{
    state.pending_tx = false;
    state.pending_transfer = {};
}

HAL_StatusTypeDef receiveFrame(FDCAN_HandleTypeDef *hfdcan,
                               FdcanBus::Frame &frame,
                               uint32_t rx_location)
{
    if (hfdcan == nullptr) {
        return HAL_ERROR;
    }

    FDCAN_RxHeaderTypeDef header = {};
    const HAL_StatusTypeDef status = HAL_FDCAN_GetRxMessage(hfdcan, rx_location, &header, frame.data);
    if (status != HAL_OK) {
        return status;
    }

    frame.id = header.Identifier;
    frame.id_type = header.IdType;
    frame.frame_type = header.RxFrameType;
    frame.data_length = header.DataLength;
    frame.error_state_indicator = header.ErrorStateIndicator;
    frame.bitrate_switch = header.BitRateSwitch;
    frame.fd_format = header.FDFormat;
    return HAL_OK;
}

void dispatchReceive(FdcanBus *bus, FDCAN_HandleTypeDef *hfdcan, uint32_t rx_location)
{
    CallbackState *state = findCallbackState(hfdcan);
    if (bus == nullptr || state == nullptr) {
        return;
    }

    const auto callbacks_it = state->receive_callbacks.find(rx_location);
    if (callbacks_it == state->receive_callbacks.end() || callbacks_it->second.empty()) {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, rx_location) != 0U) {
        FdcanBus::Frame frame = {};
        if (receiveFrame(hfdcan, frame, rx_location) != HAL_OK) {
            return;
        }

        for (const auto &callback : callbacks_it->second) {
            if (callback) {
                callback(frame);
            }
        }
    }
}

void dispatchPendingTransfer(FDCAN_HandleTypeDef *hfdcan,
                             CallbackState &state,
                             uint32_t buffer_indexes)
{
    if (!state.pending_tx) {
        return;
    }

    const uint32_t latest_buffer = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(hfdcan);
    if ((latest_buffer == 0U) || ((buffer_indexes & latest_buffer) == 0U)) {
        return;
    }

    TxTransfer transfer = std::move(state.pending_transfer);
    clearPendingTransfer(state);
    if (transfer.finished_callback) {
        transfer.finished_callback(static_cast<void *>(&transfer.frame));
    }
}

void dispatchCompletedTransfers(FDCAN_HandleTypeDef *hfdcan, uint32_t buffer_indexes)
{
    CallbackState *state = findCallbackState(hfdcan);
    if (state == nullptr) {
        return;
    }

    dispatchPendingTransfer(hfdcan, *state, buffer_indexes);

    uint32_t remaining_indexes = buffer_indexes;
    while (remaining_indexes != 0U) {
        const uint32_t buffer_index = remaining_indexes & (~remaining_indexes + 1U);
        remaining_indexes &= ~buffer_index;

        auto state_it = callback_states.find(hfdcan);
        if (state_it == callback_states.end()) {
            return;
        }

        auto transfer_it = state_it->second.tx_transfers.find(buffer_index);
        if (transfer_it == state_it->second.tx_transfers.end()) {
            continue;
        }

        TxTransfer transfer = std::move(transfer_it->second);
        state_it->second.tx_transfers.erase(transfer_it);
        if (transfer.finished_callback) {
            transfer.finished_callback(static_cast<void *>(&transfer.frame));
        }
    }
}

} // namespace

std::unordered_map<FDCAN_HandleTypeDef *, FdcanBus *> FdcanBus::handle_map;

HAL_StatusTypeDef FdcanBus::configFilter(const FDCAN_FilterTypeDef &filter) const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }
    return HAL_FDCAN_ConfigFilter(handle_, &filter);
}

HAL_StatusTypeDef FdcanBus::configGlobalFilter(uint32_t non_matching_std,
                                               uint32_t non_matching_ext,
                                               uint32_t reject_remote_std,
                                               uint32_t reject_remote_ext) const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }
    return HAL_FDCAN_ConfigGlobalFilter(handle_, non_matching_std, non_matching_ext,
                                        reject_remote_std, reject_remote_ext);
}

HAL_StatusTypeDef FdcanBus::configFifoWatermark(uint32_t fifo, uint32_t watermark) const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }
    return HAL_FDCAN_ConfigFifoWatermark(handle_, fifo, watermark);
}

HAL_StatusTypeDef FdcanBus::configAcceptAllFilters(uint32_t std_filter_index,
                                                   uint32_t ext_filter_index,
                                                   uint32_t fifo) const
{
    FDCAN_FilterTypeDef std_filter = {};
    std_filter.IdType = FDCAN_STANDARD_ID;
    std_filter.FilterIndex = std_filter_index;
    std_filter.FilterType = FDCAN_FILTER_MASK;
    std_filter.FilterID1 = 0x000U;
    std_filter.FilterID2 = 0x000U;
    std_filter.FilterConfig = fifo;

    HAL_StatusTypeDef status = configFilter(std_filter);
    if (status != HAL_OK) {
        return status;
    }

    FDCAN_FilterTypeDef ext_filter = {};
    ext_filter.IdType = FDCAN_EXTENDED_ID;
    ext_filter.FilterIndex = ext_filter_index;
    ext_filter.FilterType = FDCAN_FILTER_MASK;
    ext_filter.FilterID1 = 0x00000000U;
    ext_filter.FilterID2 = 0x00000000U;
    ext_filter.FilterConfig = fifo;

    status = configFilter(ext_filter);
    if (status != HAL_OK) {
        return status;
    }

    return configGlobalFilter();
}

HAL_StatusTypeDef FdcanBus::start(uint32_t active_its, uint32_t rx_fifo0_watermark) const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status = registerCallbacks();
    if (status != HAL_OK) {
        return status;
    }

    status = configFifoWatermark(FDCAN_CFG_RX_FIFO0, rx_fifo0_watermark);
    if (status != HAL_OK) {
        return status;
    }

    status = HAL_FDCAN_ActivateNotification(handle_, active_its, 0xFFFFFFFFU);
    if (status != HAL_OK) {
        return status;
    }

    return handle_ != nullptr ? HAL_FDCAN_Start(handle_) : HAL_ERROR;
}

HAL_StatusTypeDef FdcanBus::stop() const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }
    return HAL_FDCAN_Stop(handle_);
}

HAL_StatusTypeDef FdcanBus::activateNotification(uint32_t active_its, uint32_t buffer_indexes) const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }
    return HAL_FDCAN_ActivateNotification(handle_, active_its, buffer_indexes);
}

HAL_StatusTypeDef FdcanBus::deactivateNotification(uint32_t inactive_its) const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }
    return HAL_FDCAN_DeactivateNotification(handle_, inactive_its);
}

HAL_StatusTypeDef FdcanBus::transmit(uint32_t id, const uint8_t *data, uint8_t length,
                                     uint32_t id_type, uint32_t fd_format,
                                     uint32_t bitrate_switch,
                                     std::function<void(void *)> cplt_cb) const
{
    if (data == nullptr || handle_ == nullptr) {
        return HAL_ERROR;
    }

    Frame frame = {};
    frame.id = id;
    frame.id_type = id_type;
    frame.data_length = lengthToDlc(length);
    frame.fd_format = fd_format;
    frame.bitrate_switch = bitrate_switch;

    const uint8_t copy_length = dlcToLength(frame.data_length);
    for (uint8_t i = 0U; i < copy_length; ++i) {
        frame.data[i] = data[i];
    }

    return transmit(frame, std::move(cplt_cb));
}

HAL_StatusTypeDef FdcanBus::transmit(const Frame &frame,
                                     std::function<void(void *)> cplt_cb) const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }

    CallbackState &state = callback_states[handle_];
    state.pending_transfer.frame = frame;
    state.pending_transfer.finished_callback = std::move(cplt_cb);
    state.pending_tx = static_cast<bool>(state.pending_transfer.finished_callback);

    if (state.pending_tx) {
        const HAL_StatusTypeDef notification_status =
            HAL_FDCAN_ActivateNotification(handle_, FDCAN_IT_TX_COMPLETE, 0xFFFFFFFFU);
        if (notification_status != HAL_OK) {
            clearPendingTransfer(state);
            return notification_status;
        }
    }

    FDCAN_TxHeaderTypeDef header = makeTxHeader(frame);
    const HAL_StatusTypeDef status =
        HAL_FDCAN_AddMessageToTxFifoQ(handle_, &header, frame.data);
    if (status != HAL_OK) {
        clearPendingTransfer(state);
        return status;
    }

    if (state.pending_tx) {
        const uint32_t buffer_index = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(handle_);
        if (buffer_index == 0U) {
            clearPendingTransfer(state);
            return HAL_ERROR;
        }
        state.tx_transfers[buffer_index] = std::move(state.pending_transfer);
        clearPendingTransfer(state);
    }

    return HAL_OK;
}

HAL_StatusTypeDef FdcanBus::register_recv_cb(std::function<void(const Frame &)> recv_cb,
                                             uint32_t rx_location) const
{
    if (handle_ == nullptr || !recv_cb ||
        (rx_location != FDCAN_RX_FIFO0 && rx_location != FDCAN_RX_FIFO1)) {
        return HAL_ERROR;
    }

    handle_map[handle_] = const_cast<FdcanBus *>(this);
    CallbackState &state = callback_states[handle_];
    state.receive_callbacks[rx_location].push_back(std::move(recv_cb));
    return HAL_OK;
}

uint32_t FdcanBus::lengthToDlc(uint8_t length)
{
    if (length <= 8U) {
        return length;
    }
    if (length <= 12U) {
        return FDCAN_DLC_BYTES_12;
    }
    if (length <= 16U) {
        return FDCAN_DLC_BYTES_16;
    }
    if (length <= 20U) {
        return FDCAN_DLC_BYTES_20;
    }
    if (length <= 24U) {
        return FDCAN_DLC_BYTES_24;
    }
    if (length <= 32U) {
        return FDCAN_DLC_BYTES_32;
    }
    if (length <= 48U) {
        return FDCAN_DLC_BYTES_48;
    }
    return FDCAN_DLC_BYTES_64;
}

uint8_t FdcanBus::dlcToLength(uint32_t dlc)
{
    switch (dlc) {
    case FDCAN_DLC_BYTES_0:
        return 0U;
    case FDCAN_DLC_BYTES_1:
        return 1U;
    case FDCAN_DLC_BYTES_2:
        return 2U;
    case FDCAN_DLC_BYTES_3:
        return 3U;
    case FDCAN_DLC_BYTES_4:
        return 4U;
    case FDCAN_DLC_BYTES_5:
        return 5U;
    case FDCAN_DLC_BYTES_6:
        return 6U;
    case FDCAN_DLC_BYTES_7:
        return 7U;
    case FDCAN_DLC_BYTES_8:
        return 8U;
    case FDCAN_DLC_BYTES_12:
        return 12U;
    case FDCAN_DLC_BYTES_16:
        return 16U;
    case FDCAN_DLC_BYTES_20:
        return 20U;
    case FDCAN_DLC_BYTES_24:
        return 24U;
    case FDCAN_DLC_BYTES_32:
        return 32U;
    case FDCAN_DLC_BYTES_48:
        return 48U;
    case FDCAN_DLC_BYTES_64:
        return 64U;
    default:
        return 0U;
    }
}

FDCAN_TxHeaderTypeDef FdcanBus::makeTxHeader(const Frame &frame)
{
    FDCAN_TxHeaderTypeDef header = {};
    header.Identifier = frame.id;
    header.IdType = frame.id_type;
    header.TxFrameType = frame.frame_type;
    header.DataLength = frame.data_length;
    header.ErrorStateIndicator = frame.error_state_indicator;
    header.BitRateSwitch = frame.bitrate_switch;
    header.FDFormat = frame.fd_format;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0x00U;
    return header;
}

HAL_StatusTypeDef FdcanBus::registerCallbacks() const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }

#if (USE_HAL_FDCAN_REGISTER_CALLBACKS == 1)
    bool registered = true;

    if (HAL_FDCAN_RegisterClockCalibrationCallback(
            handle_, &FdcanBus::clockCalibrationCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_FDCAN_RegisterTxEventFifoCallback(
            handle_, &FdcanBus::txEventFifoCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_FDCAN_RegisterRxFifo0Callback(
            handle_, &FdcanBus::rxFifo0Callback) != HAL_OK) {
        registered = false;
    }
    if (HAL_FDCAN_RegisterRxFifo1Callback(
            handle_, &FdcanBus::rxFifo1Callback) != HAL_OK) {
        registered = false;
    }
    if (HAL_FDCAN_RegisterCallback(
            handle_, HAL_FDCAN_TX_FIFO_EMPTY_CB_ID, &FdcanBus::txFifoEmptyCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_FDCAN_RegisterTxBufferCompleteCallback(
            handle_, &FdcanBus::txBufferCompleteCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_FDCAN_RegisterTxBufferAbortCallback(
            handle_, &FdcanBus::txBufferAbortCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_FDCAN_RegisterCallback(
            handle_, HAL_FDCAN_RX_BUFFER_NEW_MSG_CB_ID,
            &FdcanBus::rxBufferNewMessageCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_FDCAN_RegisterCallback(
            handle_, HAL_FDCAN_HIGH_PRIO_MESSAGE_CB_ID,
            &FdcanBus::highPriorityMessageCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_FDCAN_RegisterCallback(
            handle_, HAL_FDCAN_TIMESTAMP_WRAPAROUND_CB_ID,
            &FdcanBus::timestampWraparoundCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_FDCAN_RegisterCallback(
            handle_, HAL_FDCAN_TIMEOUT_OCCURRED_CB_ID,
            &FdcanBus::timeoutOccurredCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_FDCAN_RegisterCallback(
            handle_, HAL_FDCAN_ERROR_CALLBACK_CB_ID, &FdcanBus::errorCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_FDCAN_RegisterErrorStatusCallback(
            handle_, &FdcanBus::errorStatusCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_FDCAN_RegisterTTScheduleSyncCallback(
            handle_, &FdcanBus::ttScheduleSyncCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_FDCAN_RegisterTTTimeMarkCallback(
            handle_, &FdcanBus::ttTimeMarkCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_FDCAN_RegisterTTStopWatchCallback(
            handle_, &FdcanBus::ttStopWatchCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_FDCAN_RegisterTTGlobalTimeCallback(
            handle_, &FdcanBus::ttGlobalTimeCallback) != HAL_OK) {
        registered = false;
    }

    if (registered) {
        handle_map[handle_] = const_cast<FdcanBus *>(this);
    }
    return registered ? HAL_OK : HAL_ERROR;
#else
    return HAL_ERROR;
#endif
}

void FdcanBus::clockCalibrationCallback(FDCAN_HandleTypeDef *hfdcan,
                                         uint32_t clk_calibration_its)
{
    (void)hfdcan;
    (void)clk_calibration_its;
}

void FdcanBus::txEventFifoCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t tx_event_fifo_its)
{
    (void)hfdcan;
    (void)tx_event_fifo_its;
}

void FdcanBus::rxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo0_its)
{
    (void)rx_fifo0_its;
    if (hfdcan == nullptr) {
        return;
    }

    const auto bus_it = handle_map.find(hfdcan);
    if (bus_it == handle_map.end()) {
        return;
    }
    dispatchReceive(bus_it->second, hfdcan, FDCAN_RX_FIFO0);
}

void FdcanBus::rxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo1_its)
{
    (void)rx_fifo1_its;
    if (hfdcan == nullptr) {
        return;
    }

    const auto bus_it = handle_map.find(hfdcan);
    if (bus_it == handle_map.end()) {
        return;
    }
    dispatchReceive(bus_it->second, hfdcan, FDCAN_RX_FIFO1);
}

void FdcanBus::txFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan)
{
    (void)hfdcan;
}

void FdcanBus::txBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t buffer_indexes)
{
    if (hfdcan == nullptr) {
        return;
    }
    dispatchCompletedTransfers(hfdcan, buffer_indexes);
}

void FdcanBus::txBufferAbortCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t buffer_indexes)
{
    if (hfdcan == nullptr) {
        return;
    }

    CallbackState *state = findCallbackState(hfdcan);
    if (state == nullptr) {
        return;
    }

    uint32_t remaining_indexes = buffer_indexes;
    while (remaining_indexes != 0U) {
        const uint32_t buffer_index = remaining_indexes & (~remaining_indexes + 1U);
        remaining_indexes &= ~buffer_index;
        state->tx_transfers.erase(buffer_index);
    }
}

void FdcanBus::rxBufferNewMessageCallback(FDCAN_HandleTypeDef *hfdcan)
{
    (void)hfdcan;
}

void FdcanBus::highPriorityMessageCallback(FDCAN_HandleTypeDef *hfdcan)
{
    (void)hfdcan;
}

void FdcanBus::timestampWraparoundCallback(FDCAN_HandleTypeDef *hfdcan)
{
    (void)hfdcan;
}

void FdcanBus::timeoutOccurredCallback(FDCAN_HandleTypeDef *hfdcan)
{
    (void)hfdcan;
}

void FdcanBus::errorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    (void)hfdcan;
}

void FdcanBus::errorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t error_status_its)
{
    (void)hfdcan;
    (void)error_status_its;
}

void FdcanBus::ttScheduleSyncCallback(FDCAN_HandleTypeDef *hfdcan,
                                       uint32_t tt_sched_sync_its)
{
    (void)hfdcan;
    (void)tt_sched_sync_its;
}

void FdcanBus::ttTimeMarkCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t tt_time_mark_its)
{
    (void)hfdcan;
    (void)tt_time_mark_its;
}

void FdcanBus::ttStopWatchCallback(FDCAN_HandleTypeDef *hfdcan,
                                    uint32_t sw_time,
                                    uint32_t sw_cycle_count)
{
    (void)hfdcan;
    (void)sw_time;
    (void)sw_cycle_count;
}

void FdcanBus::ttGlobalTimeCallback(FDCAN_HandleTypeDef *hfdcan,
                                     uint32_t tt_global_time_its)
{
    (void)hfdcan;
    (void)tt_global_time_its;
}

} // namespace bsp
