#pragma once

#include "BMI088driver.h"
#include "bmi088.hpp"
#include "imubase.hpp"

#include <Eigen/Dense>

class Bmi088IMU final : public IMUBase {
public:
    explicit Bmi088IMU(bmi088::Bmi088& sensor, float acceleration_scale = 9.80665F);

    void init() override;
    bool is_ready() override;
    bool update(Eigen::Quaternionf& q,
                Eigen::Vector3d& angular,
                Eigen::Vector3d& acc,
                const float& dt) override;

    bmi088::Bmi088& sensor() { return sensor_; }
    const bmi088::Bmi088& sensor() const { return sensor_; }
    uint8_t init_error() const { return init_error_; }

private:
    bmi088::Bmi088& sensor_;
    float acceleration_scale_;
    uint8_t init_error_ = BMI088_NO_ERROR;
    bool initialized_ = false;
};
