#include "usart.hpp"

#include <utility>

namespace bsp {

namespace {

using FinishedCallback = std::function<void(const void *, uint16_t)>;

struct TransferState {
    const void *tx_data = nullptr;
    uint16_t tx_size = 0;
    FinishedCallback tx_finished;
    bool tx_pending = false;
    bool tx_circular = false;

    void *rx_data = nullptr;
    uint16_t rx_size = 0;
    FinishedCallback rx_finished;
    bool rx_pending = false;
    bool rx_to_idle = false;
    bool rx_circular = false;
};

std::unordered_map<UART_HandleTypeDef *, TransferState> transfer_states;

HAL_StatusTypeDef invalidIfNull(const void *ptr)
{
    return ptr == nullptr ? HAL_ERROR : HAL_OK;
}

void clearTransmitState(TransferState &state)
{
    state.tx_pending = false;
    state.tx_circular = false;
    state.tx_data = nullptr;
    state.tx_size = 0;
    state.tx_finished = {};
}

void clearReceiveState(TransferState &state)
{
    state.rx_pending = false;
    state.rx_to_idle = false;
    state.rx_circular = false;
    state.rx_data = nullptr;
    state.rx_size = 0;
    state.rx_finished = {};
}

void eraseStateIfIdle(UART_HandleTypeDef *huart)
{
    const auto it = transfer_states.find(huart);
    if (it != transfer_states.end() && !it->second.tx_pending && !it->second.rx_pending) {
        transfer_states.erase(it);
    }
}

void completeTransmit(UART_HandleTypeDef *huart)
{
    const auto it = transfer_states.find(huart);
    if (it == transfer_states.end() || !it->second.tx_pending) {
        return;
    }

    TransferState &state = it->second;
    const void *data = state.tx_data;
    const uint16_t size = state.tx_size;

    if (state.tx_circular) {
        if (state.tx_finished) {
            state.tx_finished(data, size);
        }
        return;
    }

    FinishedCallback finished_cb = std::move(state.tx_finished);
    clearTransmitState(state);
    eraseStateIfIdle(huart);

    if (finished_cb) {
        finished_cb(data, size);
    }
}

void completeReceive(UART_HandleTypeDef *huart, uint16_t size)
{
    const auto it = transfer_states.find(huart);
    if (it == transfer_states.end() || !it->second.rx_pending) {
        return;
    }

    TransferState &state = it->second;
    void *data = state.rx_data;

    if (state.rx_circular) {
        if (state.rx_finished) {
            state.rx_finished(data, size);
        }
        return;
    }

    FinishedCallback finished_cb = std::move(state.rx_finished);
    clearReceiveState(state);
    eraseStateIfIdle(huart);

    if (finished_cb) {
        finished_cb(data, size);
    }
}

void clearAllStates(UART_HandleTypeDef *huart)
{
    const auto it = transfer_states.find(huart);
    if (it == transfer_states.end()) {
        return;
    }

    clearTransmitState(it->second);
    clearReceiveState(it->second);
    transfer_states.erase(it);
}

} // namespace

std::unordered_map<UART_HandleTypeDef *, UartPort *> UartPort::handle_map;

bool UartPort::init()
{
    if (handle_ == nullptr) {
        return false;
    }

#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
    bool registered = true;

    if (HAL_UART_RegisterCallback(handle_, HAL_UART_TX_HALFCOMPLETE_CB_ID,
                                  &UartPort::txHalfCpltCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_UART_RegisterCallback(handle_, HAL_UART_TX_COMPLETE_CB_ID,
                                  &UartPort::txCpltCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_UART_RegisterCallback(handle_, HAL_UART_RX_HALFCOMPLETE_CB_ID,
                                  &UartPort::rxHalfCpltCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_UART_RegisterCallback(handle_, HAL_UART_RX_COMPLETE_CB_ID,
                                  &UartPort::rxCpltCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_UART_RegisterCallback(handle_, HAL_UART_ERROR_CB_ID,
                                  &UartPort::errorCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_UART_RegisterCallback(handle_, HAL_UART_ABORT_COMPLETE_CB_ID,
                                  &UartPort::abortCpltCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_UART_RegisterCallback(handle_, HAL_UART_ABORT_TRANSMIT_COMPLETE_CB_ID,
                                  &UartPort::abortTransmitCpltCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_UART_RegisterCallback(handle_, HAL_UART_ABORT_RECEIVE_COMPLETE_CB_ID,
                                  &UartPort::abortReceiveCpltCallback) != HAL_OK) {
        registered = false;
    }
    if (HAL_UART_RegisterRxEventCallback(handle_, &UartPort::rxEventCallback) != HAL_OK) {
        registered = false;
    }

    if (!registered) {
        return false;
    }

    handle_map[handle_] = this;
    transfer_states.erase(handle_);
    return true;
#else
    return false;
#endif
}

void UartPort::txHalfCpltCallback(UART_HandleTypeDef *huart)
{
    (void)huart;
}

void UartPort::txCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == nullptr) {
        return;
    }

    const auto port_it = handle_map.find(huart);
    if (port_it == handle_map.end() || port_it->second == nullptr) {
        return;
    }

    completeTransmit(huart);
}

void UartPort::rxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    (void)huart;
}

void UartPort::rxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == nullptr) {
        return;
    }

    const auto port_it = handle_map.find(huart);
    if (port_it == handle_map.end() || port_it->second == nullptr) {
        return;
    }

    const auto state_it = transfer_states.find(huart);
    if (state_it == transfer_states.end()) {
        return;
    }

    completeReceive(huart, state_it->second.rx_size);
}

void UartPort::errorCallback(UART_HandleTypeDef *huart)
{
    if (huart == nullptr) {
        return;
    }

    const auto port_it = handle_map.find(huart);
    if (port_it == handle_map.end() || port_it->second == nullptr) {
        return;
    }

    clearAllStates(huart);
}

void UartPort::abortCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == nullptr) {
        return;
    }

    const auto port_it = handle_map.find(huart);
    if (port_it == handle_map.end() || port_it->second == nullptr) {
        return;
    }

    clearAllStates(huart);
}

void UartPort::abortTransmitCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == nullptr) {
        return;
    }

    const auto port_it = handle_map.find(huart);
    if (port_it == handle_map.end() || port_it->second == nullptr) {
        return;
    }

    const auto state_it = transfer_states.find(huart);
    if (state_it == transfer_states.end()) {
        return;
    }

    clearTransmitState(state_it->second);
    eraseStateIfIdle(huart);
}

void UartPort::abortReceiveCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == nullptr) {
        return;
    }

    const auto port_it = handle_map.find(huart);
    if (port_it == handle_map.end() || port_it->second == nullptr) {
        return;
    }

    const auto state_it = transfer_states.find(huart);
    if (state_it == transfer_states.end()) {
        return;
    }

    clearReceiveState(state_it->second);
    eraseStateIfIdle(huart);
}

void UartPort::rxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart == nullptr) {
        return;
    }

    const auto port_it = handle_map.find(huart);
    if (port_it == handle_map.end() || port_it->second == nullptr) {
        return;
    }

    completeReceive(huart, size);
}

HAL_StatusTypeDef UartPort::transmitIt(
    const void *data, uint16_t size, std::function<void(const void *data, uint16_t size)> finished_cb) const
{
    if (handle_ == nullptr || invalidIfNull(data) != HAL_OK || size == 0U) {
        return HAL_ERROR;
    }
    if (handle_->gState != HAL_UART_STATE_READY) {
        return HAL_BUSY;
    }

    TransferState &state = transfer_states[handle_];
    state.tx_data = data;
    state.tx_size = size;
    state.tx_finished = std::move(finished_cb);
    state.tx_pending = true;
    state.tx_circular = false;

    const HAL_StatusTypeDef status =
        HAL_UART_Transmit_IT(handle_, static_cast<const uint8_t *>(data), size);
    if (status != HAL_OK) {
        clearTransmitState(state);
        eraseStateIfIdle(handle_);
    }
    return status;
}

HAL_StatusTypeDef UartPort::receiveIt(
    void *data, uint16_t size, std::function<void(const void *data, uint16_t size)> finished_cb) const
{
    if (handle_ == nullptr || invalidIfNull(data) != HAL_OK || size == 0U) {
        return HAL_ERROR;
    }
    if (handle_->RxState != HAL_UART_STATE_READY) {
        return HAL_BUSY;
    }

    TransferState &state = transfer_states[handle_];
    state.rx_data = data;
    state.rx_size = size;
    state.rx_finished = std::move(finished_cb);
    state.rx_pending = true;
    state.rx_to_idle = false;
    state.rx_circular = false;

    const HAL_StatusTypeDef status =
        HAL_UART_Receive_IT(handle_, static_cast<uint8_t *>(data), size);
    if (status != HAL_OK) {
        clearReceiveState(state);
        eraseStateIfIdle(handle_);
    }
    return status;
}

HAL_StatusTypeDef UartPort::transmitDma(
    const void *data, uint16_t size, std::function<void(const void *data, uint16_t size)> finished_cb) const
{
    if (handle_ == nullptr || invalidIfNull(data) != HAL_OK || size == 0U) {
        return HAL_ERROR;
    }
    if (handle_->gState != HAL_UART_STATE_READY) {
        return HAL_BUSY;
    }

    TransferState &state = transfer_states[handle_];
    state.tx_data = data;
    state.tx_size = size;
    state.tx_finished = std::move(finished_cb);
    state.tx_pending = true;
    state.tx_circular = handle_->hdmatx != nullptr &&
                        handle_->hdmatx->Init.Mode == DMA_CIRCULAR;

    const HAL_StatusTypeDef status =
        HAL_UART_Transmit_DMA(handle_, static_cast<const uint8_t *>(data), size);
    if (status != HAL_OK) {
        clearTransmitState(state);
        eraseStateIfIdle(handle_);
    }
    return status;
}

HAL_StatusTypeDef UartPort::receiveDma(
    void *data, uint16_t size, std::function<void(const void *data, uint16_t size)> finished_cb) const
{
    if (handle_ == nullptr || invalidIfNull(data) != HAL_OK || size == 0U) {
        return HAL_ERROR;
    }
    if (handle_->RxState != HAL_UART_STATE_READY) {
        return HAL_BUSY;
    }

    TransferState &state = transfer_states[handle_];
    state.rx_data = data;
    state.rx_size = size;
    state.rx_finished = std::move(finished_cb);
    state.rx_pending = true;
    state.rx_to_idle = false;
    state.rx_circular = handle_->hdmarx != nullptr &&
                        handle_->hdmarx->Init.Mode == DMA_CIRCULAR;

    const HAL_StatusTypeDef status =
        HAL_UART_Receive_DMA(handle_, static_cast<uint8_t *>(data), size);
    if (status != HAL_OK) {
        clearReceiveState(state);
        eraseStateIfIdle(handle_);
    }
    return status;
}

HAL_StatusTypeDef UartPort::receiveToIdleDma(
    void *data, uint16_t size,
    std::function<void(const void *data, uint16_t size)> finished_cb,
    bool disable_half_transfer_it) const
{
    if (handle_ == nullptr || invalidIfNull(data) != HAL_OK || size == 0U) {
        return HAL_ERROR;
    }
    if (handle_->RxState != HAL_UART_STATE_READY) {
        return HAL_BUSY;
    }

    TransferState &state = transfer_states[handle_];
    state.rx_data = data;
    state.rx_size = size;
    state.rx_finished = std::move(finished_cb);
    state.rx_pending = true;
    state.rx_to_idle = true;
    state.rx_circular = handle_->hdmarx != nullptr &&
                        handle_->hdmarx->Init.Mode == DMA_CIRCULAR;

    const HAL_StatusTypeDef status =
        HAL_UARTEx_ReceiveToIdle_DMA(handle_, static_cast<uint8_t *>(data), size);
    if (status != HAL_OK) {
        clearReceiveState(state);
        eraseStateIfIdle(handle_);
        return status;
    }

    if (disable_half_transfer_it && handle_->hdmarx != nullptr) {
        __HAL_DMA_DISABLE_IT(handle_->hdmarx, DMA_IT_HT);
    }

    return HAL_OK;
}

HAL_StatusTypeDef UartPort::stopDma() const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }

    const HAL_StatusTypeDef status = HAL_UART_DMAStop(handle_);
    if (status == HAL_OK) {
        clearAllStates(handle_);
    }
    return status;
}

HAL_StatusTypeDef UartPort::abort() const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }

    const HAL_StatusTypeDef status = HAL_UART_Abort(handle_);
    if (status == HAL_OK) {
        clearAllStates(handle_);
    }
    return status;
}

HAL_StatusTypeDef UartPort::abortTransmit() const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }

    const HAL_StatusTypeDef status = HAL_UART_AbortTransmit(handle_);
    if (status == HAL_OK) {
        const auto state_it = transfer_states.find(handle_);
        if (state_it != transfer_states.end()) {
            clearTransmitState(state_it->second);
            eraseStateIfIdle(handle_);
        }
    }
    return status;
}

HAL_StatusTypeDef UartPort::abortReceive() const
{
    if (handle_ == nullptr) {
        return HAL_ERROR;
    }

    const HAL_StatusTypeDef status = HAL_UART_AbortReceive(handle_);
    if (status == HAL_OK) {
        const auto state_it = transfer_states.find(handle_);
        if (state_it != transfer_states.end()) {
            clearReceiveState(state_it->second);
            eraseStateIfIdle(handle_);
        }
    }
    return status;
}

} // namespace bsp
