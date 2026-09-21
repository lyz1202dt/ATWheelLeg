#pragma once

#include "stm32h7xx_hal.h"
#include <unordered_map>
#include <functional>

namespace bsp {

class UartPort {
public:
    explicit constexpr UartPort(UART_HandleTypeDef &handle) : handle_(&handle) {}
    bool init();

    HAL_StatusTypeDef transmitIt(const void *data, uint16_t size,std::function<void(const void *data, uint16_t size)> finished_cb) const;
    HAL_StatusTypeDef receiveIt(void *data, uint16_t size,std::function<void(const void *data, uint16_t size)> finished_cb) const;

    HAL_StatusTypeDef transmitDma(const void *data, uint16_t size,std::function<void(const void *data, uint16_t size)> finished_cb) const;
    HAL_StatusTypeDef receiveDma(void *data, uint16_t size,std::function<void(const void *data, uint16_t size)> finished_cb) const;
    HAL_StatusTypeDef receiveToIdleDma(void *data, uint16_t size,std::function<void(const void *data, uint16_t size)> finished_cb,bool disable_half_transfer_it = true) const;

    HAL_StatusTypeDef stopDma() const;
    HAL_StatusTypeDef abort() const;
    HAL_StatusTypeDef abortTransmit() const;
    HAL_StatusTypeDef abortReceive() const;

private:
    static void txHalfCpltCallback(UART_HandleTypeDef *huart);
    static void txCpltCallback(UART_HandleTypeDef *huart);
    static void rxHalfCpltCallback(UART_HandleTypeDef *huart);
    static void rxCpltCallback(UART_HandleTypeDef *huart);
    static void errorCallback(UART_HandleTypeDef *huart);
    static void abortCpltCallback(UART_HandleTypeDef *huart);
    static void abortTransmitCpltCallback(UART_HandleTypeDef *huart);
    static void abortReceiveCpltCallback(UART_HandleTypeDef *huart);
    static void rxEventCallback(UART_HandleTypeDef *huart, uint16_t size);

    //HAL句柄和类接口绑定，静态回调中通过这个区分不同的设备
    static std::unordered_map<UART_HandleTypeDef*, UartPort*> handle_map;
    UART_HandleTypeDef *handle_ = nullptr;
};

} // namespace bsp
