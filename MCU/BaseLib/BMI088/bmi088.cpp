#include "bmi088.hpp"

#include <math.h>
#include <string.h>

#include "BMI088driver.h"
#include "BMI088reg.h"
#include "FreeRTOS.h"
#include "task.h"

namespace bmi088 {

namespace {

constexpr uint32_t kSpiTimeout = 1000U;
constexpr uint16_t kMaxSpiFrameSize = 16U;

struct RegisterConfig {
    uint8_t reg;
    uint8_t value;
    uint8_t error;
};

constexpr RegisterConfig kAccelConfig[] = {
    {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
    {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
    {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set,
     BMI088_ACC_CONF_ERROR},
    {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},
    {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP |
                              BMI088_ACC_INT1_GPIO_LOW,
     BMI088_INT1_IO_CTRL_ERROR},
    {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR},
};

constexpr RegisterConfig kGyroConfig[] = {
    {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
    {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set,
     BMI088_GYRO_BANDWIDTH_ERROR},
    {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
    {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
    {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW,
     BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
    {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3,
     BMI088_GYRO_INT3_INT4_IO_MAP_ERROR},
};

int16_t makeInt16(uint8_t low, uint8_t high)
{
    return static_cast<int16_t>((static_cast<uint16_t>(high) << 8U) | low);
}

} // namespace

Bmi088::Bmi088(bsp::SpiBus &bus,
               bsp::GpioPin &accel_cs,
               bsp::GpioPin &gyro_cs,
               bsp::PwmChannel heater_pwm,
               bsp::AdcChannel mcu_temperature_adc)
    : Bmi088(bus, accel_cs, gyro_cs, heater_pwm, mcu_temperature_adc, Config{})
{
}

Bmi088::Bmi088(bsp::SpiBus &bus,
               bsp::GpioPin &accel_cs,
               bsp::GpioPin &gyro_cs,
               bsp::PwmChannel heater_pwm,
               bsp::AdcChannel mcu_temperature_adc,
               Config config)
    : bus_(&bus),
      accel_cs_(&accel_cs),
      gyro_cs_(&gyro_cs),
      heater_pwm_(heater_pwm),
      mcu_temperature_adc_(mcu_temperature_adc),
      config_(config),
      temperature_pid_(config.heater_kp, config.heater_kd, config.heater_ki,
                       config.heater_integral_limit, config.heater_output_limit,
                       config.task_dt * static_cast<float>(config.temperature_control_divider))
{
}

uint8_t Bmi088::init()
{
    resetRuntimeState();
    deselect(Device::Accel);
    deselect(Device::Gyro);
    (void)heater_pwm_.start();

    last_error_ = initAccel();
    last_error_ |= initGyro();

    IMU_QuaternionEKF_Init_Instance(&ekf_,
                                    config_.ekf_process_noise_q,
                                    config_.ekf_process_noise_bias,
                                    config_.ekf_measure_noise,
                                    config_.ekf_lambda,
                                    config_.task_dt,
                                    config_.ekf_accel_lpf);

    state_ = (last_error_ == BMI088_NO_ERROR) ? State::Heating : State::Error;
    return last_error_;
}

bool Bmi088::update(ImuData *data)
{
    if (state_ == State::Uninitialized || state_ == State::Error) {
        return false;
    }

    if (read(sample_) != HAL_OK) {
        last_error_ = BMI088_NO_SENSOR;
        state_ = State::Error;
        return false;
    }

    updateTemperatureFilter();

    if (state_ == State::Ready) {
        runAttitudeUpdate(data);
    } else if (state_ == State::Calibrating) {
        updateCalibration();
    }

    const uint32_t divider = config_.temperature_control_divider == 0U
                                 ? 1U
                                 : config_.temperature_control_divider;
    if ((task_count_ % divider) == 0U) {
        (void)updateTemperatureControl();
        (void)readMcuTemperature();
        maybeAdvanceTemperatureState();
    }

    ++task_count_;
    return true;
}

HAL_StatusTypeDef Bmi088::read(SensorSample &sample) const
{
    uint8_t buffer[6] = {0U};
    uint8_t temp_buffer[2] = {0U};
    int16_t raw = 0;

    HAL_StatusTypeDef status = readRegs(Device::Accel, BMI088_ACCEL_XOUT_L,
                                        buffer, sizeof(buffer), true);
    if (status != HAL_OK) {
        return status;
    }

    raw = makeInt16(buffer[0], buffer[1]);
    sample.accel[0] = static_cast<float>(raw) * BMI088_ACCEL_3G_SEN;
    raw = makeInt16(buffer[2], buffer[3]);
    sample.accel[1] = static_cast<float>(raw) * BMI088_ACCEL_3G_SEN;
    raw = makeInt16(buffer[4], buffer[5]);
    sample.accel[2] = static_cast<float>(raw) * BMI088_ACCEL_3G_SEN;

    status = readRegs(Device::Gyro, BMI088_GYRO_X_L, buffer, sizeof(buffer), false);
    if (status != HAL_OK) {
        return status;
    }

    raw = makeInt16(buffer[0], buffer[1]);
    sample.gyro[0] = static_cast<float>(raw) * BMI088_GYRO_2000_SEN;
    raw = makeInt16(buffer[2], buffer[3]);
    sample.gyro[1] = static_cast<float>(raw) * BMI088_GYRO_2000_SEN;
    raw = makeInt16(buffer[4], buffer[5]);
    sample.gyro[2] = static_cast<float>(raw) * BMI088_GYRO_2000_SEN;

    status = readRegs(Device::Accel, BMI088_TEMP_M, temp_buffer, sizeof(temp_buffer), true);
    if (status != HAL_OK) {
        return status;
    }

    raw = static_cast<int16_t>((static_cast<uint16_t>(temp_buffer[0]) << 3U) |
                               (static_cast<uint16_t>(temp_buffer[1]) >> 5U));
    if (raw > 1023) {
        raw = static_cast<int16_t>(raw - 2048);
    }
    sample.temperature = static_cast<float>(raw) * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

    return HAL_OK;
}

uint32_t Bmi088::updateTemperatureControl()
{
    const float output = temperature_pid_.update(temperature(), config_.target_temperature);
    float clamped = output;

    if (clamped < 0.0f) {
        clamped = 0.0f;
    }
    if (clamped > config_.heater_output_limit) {
        clamped = config_.heater_output_limit;
    }

    heater_output_ = static_cast<uint32_t>(clamped);
    heater_pwm_.setPulse(heater_output_);
    return heater_output_;
}

float Bmi088::readMcuTemperature()
{
    if (mcu_temp_pending_) {
        return mcu_temperature_;
    }

    mcu_temp_raw_ = 0U;
    mcu_temp_pending_ = true;
    if (mcu_temperature_adc_.startDma(&mcu_temp_raw_, 1U,
                                      [this]() { mcu_temp_pending_ = false; }) != HAL_OK) {
        mcu_temp_pending_ = false;
        return mcu_temperature_;
    }

    uint32_t timeout = 100000U;
    while (mcu_temp_pending_ && timeout-- != 0U) {
    }
    if (mcu_temp_pending_) {
        return mcu_temperature_;
    }

    const uint32_t raw = mcu_temp_raw_;

    const uint16_t ts_cal1 = *reinterpret_cast<const uint16_t *>(0x1FF1E820UL);
    const uint16_t ts_cal2 = *reinterpret_cast<const uint16_t *>(0x1FF1E840UL);
    if (ts_cal2 == ts_cal1) {
        return mcu_temperature_;
    }

    const float slope = (110.0f - 30.0f) /
                        static_cast<float>(static_cast<int32_t>(ts_cal2) -
                                           static_cast<int32_t>(ts_cal1));
    mcu_temperature_ = slope *
                           (static_cast<float>(raw) - static_cast<float>(ts_cal1)) +
                       30.0f;
    return mcu_temperature_;
}

void Bmi088::fill(ImuData &data) const
{
    IMU_QuaternionEKF_GetQuaternion(&ekf_, data.q);
    IMU_QuaternionEKF_GetAngularVelocity(&ekf_, data.gyro);
    IMU_QuaternionEKF_GetAngularAcceleration(&ekf_, data.angular_accel);
}

void Bmi088::resetRuntimeState()
{
    task_count_ = 0U;
    temperature_ready_count_ = 0U;
    calibration_count_ = 0U;
    calibration_temperature_ = 0.0f;
    heater_output_ = 0U;
    mcu_temperature_ = 0.0f;
    mcu_temp_raw_ = 0U;
    mcu_temp_pending_ = false;
    motion_filter_ready_ = false;
    temperature_filter_ready_ = false;
    temperature_filtered_ = 0.0f;
    sample_ = {};
    temperature_pid_.reset();

    for (uint8_t i = 0U; i < 3U; ++i) {
        gyro_bias_[i] = 0.0f;
        gyro_sum_[i] = 0.0f;
        gyro_corrected_[i] = 0.0f;
        gyro_filtered_[i] = 0.0f;
        accel_filtered_[i] = 0.0f;
    }
}

uint8_t Bmi088::initAccel()
{
    uint8_t chip_id = 0U;

    if (!deviceValid(Device::Accel)) {
        return BMI088_NO_SENSOR;
    }

    (void)readReg(Device::Accel, BMI088_ACC_CHIP_ID, chip_id, true);
    delayUs(BMI088_COM_WAIT_SENSOR_TIME);
    (void)readReg(Device::Accel, BMI088_ACC_CHIP_ID, chip_id, true);
    delayUs(BMI088_COM_WAIT_SENSOR_TIME);

    if (writeReg(Device::Accel, BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE) != HAL_OK) {
        return BMI088_NO_SENSOR;
    }
    delayMs(BMI088_LONG_DELAY_TIME);

    (void)readReg(Device::Accel, BMI088_ACC_CHIP_ID, chip_id, true);
    delayUs(BMI088_COM_WAIT_SENSOR_TIME);
    (void)readReg(Device::Accel, BMI088_ACC_CHIP_ID, chip_id, true);
    delayUs(BMI088_COM_WAIT_SENSOR_TIME);

    if (chip_id != BMI088_ACC_CHIP_ID_VALUE) {
        return BMI088_NO_SENSOR;
    }

    for (const auto &reg_config : kAccelConfig) {
        uint8_t read_back = 0U;
        if (writeReg(Device::Accel, reg_config.reg, reg_config.value) != HAL_OK) {
            return reg_config.error;
        }
        delayUs(BMI088_COM_WAIT_SENSOR_TIME);
        if (readReg(Device::Accel, reg_config.reg, read_back, true) != HAL_OK) {
            return reg_config.error;
        }
        delayUs(BMI088_COM_WAIT_SENSOR_TIME);
        if (read_back != reg_config.value) {
            return reg_config.error;
        }
    }

    return BMI088_NO_ERROR;
}

uint8_t Bmi088::initGyro()
{
    uint8_t chip_id = 0U;

    if (!deviceValid(Device::Gyro)) {
        return BMI088_NO_SENSOR;
    }

    (void)readReg(Device::Gyro, BMI088_GYRO_CHIP_ID, chip_id, false);
    delayUs(BMI088_COM_WAIT_SENSOR_TIME);
    (void)readReg(Device::Gyro, BMI088_GYRO_CHIP_ID, chip_id, false);
    delayUs(BMI088_COM_WAIT_SENSOR_TIME);

    if (writeReg(Device::Gyro, BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE) != HAL_OK) {
        return BMI088_NO_SENSOR;
    }
    delayMs(BMI088_LONG_DELAY_TIME);

    (void)readReg(Device::Gyro, BMI088_GYRO_CHIP_ID, chip_id, false);
    delayUs(BMI088_COM_WAIT_SENSOR_TIME);
    (void)readReg(Device::Gyro, BMI088_GYRO_CHIP_ID, chip_id, false);
    delayUs(BMI088_COM_WAIT_SENSOR_TIME);

    if (chip_id != BMI088_GYRO_CHIP_ID_VALUE) {
        return BMI088_NO_SENSOR;
    }

    for (const auto &reg_config : kGyroConfig) {
        uint8_t read_back = 0U;
        if (writeReg(Device::Gyro, reg_config.reg, reg_config.value) != HAL_OK) {
            return reg_config.error;
        }
        delayUs(BMI088_COM_WAIT_SENSOR_TIME);
        if (readReg(Device::Gyro, reg_config.reg, read_back, false) != HAL_OK) {
            return reg_config.error;
        }
        delayUs(BMI088_COM_WAIT_SENSOR_TIME);
        if (read_back != reg_config.value) {
            return reg_config.error;
        }
    }

    return BMI088_NO_ERROR;
}

void Bmi088::enterCalibration()
{
    state_ = State::Calibrating;
    calibration_count_ = 0U;
    calibration_temperature_ = temperature();
    motion_filter_ready_ = false;

    for (uint8_t i = 0U; i < 3U; ++i) {
        gyro_sum_[i] = 0.0f;
        gyro_bias_[i] = 0.0f;
        gyro_filtered_[i] = 0.0f;
        accel_filtered_[i] = 0.0f;
    }
}

void Bmi088::updateCalibration()
{
    gyro_sum_[0] += sample_.gyro[0];
    gyro_sum_[1] += sample_.gyro[1];
    gyro_sum_[2] += sample_.gyro[2];
    ++calibration_count_;

    if (calibration_count_ < config_.gyro_calibration_samples) {
        return;
    }

    const float inv_count = 1.0f / static_cast<float>(config_.gyro_calibration_samples);
    gyro_bias_[0] = gyro_sum_[0] * inv_count;
    gyro_bias_[1] = gyro_sum_[1] * inv_count;
    gyro_bias_[2] = gyro_sum_[2] * inv_count;
    motion_filter_ready_ = false;
    IMU_QuaternionEKF_Reset_Instance(&ekf_);
    state_ = State::Ready;
}

void Bmi088::updateMotionFilters()
{
    if (!motion_filter_ready_) {
        gyro_filtered_[0] = gyro_corrected_[0];
        gyro_filtered_[1] = gyro_corrected_[1];
        gyro_filtered_[2] = gyro_corrected_[2];
        accel_filtered_[0] = sample_.accel[0];
        accel_filtered_[1] = sample_.accel[1];
        accel_filtered_[2] = sample_.accel[2];
        motion_filter_ready_ = true;
        return;
    }

    gyro_filtered_[0] = lowPass(gyro_filtered_[0], gyro_corrected_[0], config_.gyro_lpf_alpha);
    gyro_filtered_[1] = lowPass(gyro_filtered_[1], gyro_corrected_[1], config_.gyro_lpf_alpha);
    gyro_filtered_[2] = lowPass(gyro_filtered_[2], gyro_corrected_[2], config_.gyro_lpf_alpha);

    accel_filtered_[0] = lowPass(accel_filtered_[0], sample_.accel[0], config_.accel_lpf_alpha);
    accel_filtered_[1] = lowPass(accel_filtered_[1], sample_.accel[1], config_.accel_lpf_alpha);
    accel_filtered_[2] = lowPass(accel_filtered_[2], sample_.accel[2], config_.accel_lpf_alpha);
}

void Bmi088::updateTemperatureFilter()
{
    if (!temperature_filter_ready_) {
        temperature_filtered_ = sample_.temperature;
        temperature_filter_ready_ = true;
        return;
    }

    temperature_filtered_ = lowPass(temperature_filtered_,
                                   sample_.temperature,
                                   config_.temperature_lpf_alpha);
}

void Bmi088::maybeAdvanceTemperatureState()
{
    if (state_ != State::Heating) {
        return;
    }

    if (fabsf(temperature() - config_.target_temperature) < config_.temperature_ready_error) {
        ++temperature_ready_count_;
        if (temperature_ready_count_ > config_.temperature_ready_ticks) {
            enterCalibration();
        }
    } else {
        temperature_ready_count_ = 0U;
    }
}

void Bmi088::runAttitudeUpdate(ImuData *data)
{
    gyro_corrected_[0] = sample_.gyro[0] - gyro_bias_[0];
    gyro_corrected_[1] = sample_.gyro[1] - gyro_bias_[1];
    gyro_corrected_[2] = sample_.gyro[2] - gyro_bias_[2];

    if (config_.gyro_z_deadband > 0.0f &&
        fabsf(gyro_corrected_[2]) < config_.gyro_z_deadband) {
        gyro_corrected_[2] = 0.0f;
    }

    updateMotionFilters();
    IMU_QuaternionEKF_Update_Instance(&ekf_,
                                      gyro_filtered_[0],
                                      gyro_filtered_[1],
                                      gyro_filtered_[2],
                                      accel_filtered_[0],
                                      accel_filtered_[1],
                                      accel_filtered_[2]);

    if (data != nullptr) {
        fill(*data);
    }
}

bool Bmi088::valid() const
{
    return bus_ != nullptr && bus_->valid() && accel_cs_ != nullptr && gyro_cs_ != nullptr;
}

bool Bmi088::deviceValid(Device device) const
{
    if (bus_ == nullptr || !bus_->valid()) {
        return false;
    }

    const bsp::GpioPin *chip_select = device == Device::Accel ? accel_cs_ : gyro_cs_;
    return chip_select != nullptr;
}

void Bmi088::select(Device device) const
{
    bsp::GpioPin *chip_select = device == Device::Accel ? accel_cs_ : gyro_cs_;
    if (chip_select != nullptr) {
        chip_select->write(GPIO_PIN_RESET);
    }
}

void Bmi088::deselect(Device device) const
{
    bsp::GpioPin *chip_select = device == Device::Accel ? accel_cs_ : gyro_cs_;
    if (chip_select != nullptr) {
        chip_select->write(GPIO_PIN_SET);
    }
}

HAL_StatusTypeDef Bmi088::transfer(Device device, const uint8_t *tx_data,
                                   uint8_t *rx_data, uint16_t size) const
{
    if (!deviceValid(device) || tx_data == nullptr || rx_data == nullptr || size == 0U) {
        return HAL_ERROR;
    }
    if (__get_IPSR() != 0U) {
        return HAL_BUSY;
    }

    struct BlockingContext {
        volatile bool done = false;
        HAL_StatusTypeDef status = HAL_BUSY;
    };

    BlockingContext context;
    bsp::SPITransfer transfer;
    transfer.tx_data = tx_data;
    transfer.rx_data = rx_data;
    transfer.size = size;
    transfer.param = &context;
    transfer.pre_transmit_cb = [this, device](bsp::SPITransfer *) {
        select(device);
    };
    transfer.transmited_cb = [this, device, &context](bsp::SPITransfer *finished) {
        context.status = finished->status;
        deselect(device);
        context.done = true;
    };

    if (!bus_->transferIt(transfer)) {
        return HAL_BUSY;
    }

    const uint32_t start_tick = HAL_GetTick();
    while (!context.done) {
        if ((HAL_GetTick() - start_tick) >= kSpiTimeout) {
            (void)bus_->abort();
            if (!context.done) {
                deselect(device);
            }
            return HAL_TIMEOUT;
        }
        if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
            taskYIELD();
        }
    }

    return context.status;
}

HAL_StatusTypeDef Bmi088::writeReg(Device device, uint8_t reg, uint8_t value) const
{
    uint8_t tx[2] = {reg, value};
    uint8_t rx[2] = {0U};

    return transfer(device, tx, rx, sizeof(tx));
}

HAL_StatusTypeDef Bmi088::readReg(Device device, uint8_t reg, uint8_t &value,
                                  bool accel_dummy) const
{
    return readRegs(device, reg, &value, 1U, accel_dummy);
}

HAL_StatusTypeDef Bmi088::readRegs(Device device, uint8_t reg, uint8_t *buffer,
                                   uint8_t length, bool accel_dummy) const
{
    if (!deviceValid(device) || buffer == nullptr) {
        return HAL_ERROR;
    }
    if (length == 0U) {
        return HAL_OK;
    }

    const uint16_t dummy_size = accel_dummy ? 1U : 0U;
    const uint16_t frame_size = static_cast<uint16_t>(1U + dummy_size + length);
    if (frame_size > kMaxSpiFrameSize) {
        return HAL_ERROR;
    }

    uint8_t tx[kMaxSpiFrameSize] = {};
    uint8_t rx[kMaxSpiFrameSize] = {};
    memset(tx, 0x55, frame_size);
    tx[0] = static_cast<uint8_t>(reg | 0x80U);

    const HAL_StatusTypeDef status = transfer(device, tx, rx, frame_size);
    if (status == HAL_OK) {
        memcpy(buffer, &rx[1U + dummy_size], length);
    }

    return status;
}

float Bmi088::lowPass(float previous, float input, float alpha)
{
    if (alpha <= 0.0f) {
        return previous;
    }
    if (alpha >= 1.0f) {
        return input;
    }
    return previous + alpha * (input - previous);
}

void Bmi088::delayMs(uint32_t ms)
{
    HAL_Delay(ms);
}

void Bmi088::delayUs(uint32_t us)
{
    if (us == 0U) {
        return;
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    const uint32_t cycles_per_us = SystemCoreClock / 1000000U;
    const uint32_t wait_cycles = cycles_per_us * us;
    const uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < wait_cycles) {
    }
}

} // namespace bmi088
