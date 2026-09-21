#pragma once

#include <stdint.h>

#include "QuaternionEKF.h"
#include "adc.hpp"
#include "gpio.hpp"
#include "pid.hpp"
#include "spi.hpp"
#include "tim.hpp"

namespace bmi088 {

struct ImuData {
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float gyro[3] = {0.0f, 0.0f, 0.0f};
    float angular_accel[3] = {0.0f, 0.0f, 0.0f};
};

class Bmi088 {
public:
    enum class State : uint8_t {
        Uninitialized,
        Heating,
        Calibrating,
        Ready,
        Error,
    };

    struct Config {
        float task_dt = 0.001f;
        uint32_t temperature_control_divider = 2U;

        float target_temperature = 47.0f;
        float temperature_ready_error = 5.0f;
        uint32_t temperature_ready_ticks = 100U;

        uint32_t gyro_calibration_samples = 1000U;
        float gyro_lpf_alpha = 0.12f;
        float accel_lpf_alpha = 0.08f;
        float temperature_lpf_alpha = 0.05f;
        float gyro_z_deadband = 0.0f;

        float heater_kp = 286.0f;
        float heater_kd = 0.0f;
        float heater_ki = 0.0f;
        float heater_integral_limit = 2000.0f;
        float heater_output_limit = 2000.0f;

        float ekf_process_noise_q = 10.0f;
        float ekf_process_noise_bias = 0.001f;
        float ekf_measure_noise = 10000000.0f;
        float ekf_lambda = 1.0f;
        float ekf_accel_lpf = 0.0f;
    };

    struct SensorSample {
        float gyro[3] = {0.0f, 0.0f, 0.0f};
        float accel[3] = {0.0f, 0.0f, 0.0f};
        float temperature = 0.0f;
    };

    Bmi088(bsp::SpiBus &bus,
           bsp::GpioPin &accel_cs,
           bsp::GpioPin &gyro_cs,
           bsp::PwmChannel heater_pwm,
           bsp::AdcChannel mcu_temperature_adc);
    Bmi088(bsp::SpiBus &bus,
           bsp::GpioPin &accel_cs,
           bsp::GpioPin &gyro_cs,
           bsp::PwmChannel heater_pwm,
           bsp::AdcChannel mcu_temperature_adc,
           Config config);

    uint8_t init();
    bool update(ImuData *data = nullptr);
    HAL_StatusTypeDef read(SensorSample &sample) const;

    uint32_t updateTemperatureControl();
    float readMcuTemperature();
    void fill(ImuData &data) const;
    void resetRuntimeState();

    State state() const { return state_; }
    uint8_t lastError() const { return last_error_; }
    const SensorSample &sample() const { return sample_; }
    const float *gyroBias() const { return gyro_bias_; }
    const float *filteredGyro() const { return gyro_filtered_; }
    const float *filteredAccel() const { return accel_filtered_; }
    float temperature() const { return temperature_filter_ready_ ? temperature_filtered_ : sample_.temperature; }
    float mcuTemperature() const { return mcu_temperature_; }
    uint32_t heaterOutput() const { return heater_output_; }
    const QEKF_INS_t &ekf() const { return ekf_; }

private:
    enum class Device : uint8_t {
        Accel,
        Gyro,
    };

    uint8_t initAccel();
    uint8_t initGyro();
    void enterCalibration();
    void updateCalibration();
    void updateMotionFilters();
    void updateTemperatureFilter();
    void maybeAdvanceTemperatureState();
    void runAttitudeUpdate(ImuData *data);

    bool valid() const;
    bool deviceValid(Device device) const;
    void select(Device device) const;
    void deselect(Device device) const;
    HAL_StatusTypeDef transfer(Device device, const uint8_t *tx_data,
                               uint8_t *rx_data, uint16_t size) const;
    HAL_StatusTypeDef writeReg(Device device, uint8_t reg, uint8_t value) const;
    HAL_StatusTypeDef readReg(Device device, uint8_t reg, uint8_t &value,
                              bool accel_dummy) const;
    HAL_StatusTypeDef readRegs(Device device, uint8_t reg, uint8_t *buffer,
                               uint8_t length, bool accel_dummy) const;

    static float lowPass(float previous, float input, float alpha);
    static void delayMs(uint32_t ms);
    static void delayUs(uint32_t us);

    bsp::SpiBus *bus_ = nullptr;
    bsp::GpioPin *accel_cs_ = nullptr;
    bsp::GpioPin *gyro_cs_ = nullptr;
    bsp::PwmChannel heater_pwm_;
    bsp::AdcChannel mcu_temperature_adc_;
    Config config_;
    PID temperature_pid_;
    QEKF_INS_t ekf_ = {};

    State state_ = State::Uninitialized;
    uint8_t last_error_ = 0U;
    uint32_t task_count_ = 0U;
    uint32_t temperature_ready_count_ = 0U;
    uint32_t calibration_count_ = 0U;
    float calibration_temperature_ = 0.0f;

    SensorSample sample_ = {};
    float gyro_bias_[3] = {0.0f, 0.0f, 0.0f};
    float gyro_sum_[3] = {0.0f, 0.0f, 0.0f};
    float gyro_corrected_[3] = {0.0f, 0.0f, 0.0f};
    float gyro_filtered_[3] = {0.0f, 0.0f, 0.0f};
    float accel_filtered_[3] = {0.0f, 0.0f, 0.0f};
    float temperature_filtered_ = 0.0f;
    float mcu_temperature_ = 0.0f;
    uint32_t mcu_temp_raw_ = 0U;
    volatile bool mcu_temp_pending_ = false;
    uint32_t heater_output_ = 0U;
    bool motion_filter_ready_ = false;
    bool temperature_filter_ready_ = false;
};

} // namespace bmi088
