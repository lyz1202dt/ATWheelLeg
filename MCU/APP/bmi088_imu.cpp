#include "bmi088_imu.hpp"

#include <cmath>

Bmi088IMU::Bmi088IMU(bmi088::Bmi088& sensor, float acceleration_scale)
    : sensor_(sensor),
      acceleration_scale_(acceleration_scale)
{
}

void Bmi088IMU::init()
{
    init_error_ = sensor_.init();
    initialized_ = init_error_ == BMI088_NO_ERROR;
}

bool Bmi088IMU::is_ready()
{
    return initialized_ && sensor_.state() == bmi088::Bmi088::State::Ready;
}

bool Bmi088IMU::update(const float& dt)
{
    (void)dt;

    if (!initialized_) {
        lock_memory();
        state_valid_ = false;
        unlock_memory();
        return false;
    }

    bmi088::ImuData attitude;
    if (!sensor_.update(&attitude)) {
        lock_memory();
        state_valid_ = false;
        unlock_memory();
        return false;
    }

    const Eigen::Quaternionf next_orientation(
        attitude.q[0],
        attitude.q[1],
        attitude.q[2],
        attitude.q[3]);
    if (!next_orientation.coeffs().allFinite() ||
        next_orientation.squaredNorm() < 1.0e-8F) {
        lock_memory();
        state_valid_ = false;
        unlock_memory();
        return false;
    }

    const Eigen::Quaternionf next_orientation_normalized =
        next_orientation.normalized();
    const Eigen::Vector3d next_angular_velocity(
        static_cast<double>(attitude.gyro[0]),
        static_cast<double>(attitude.gyro[1]),
        static_cast<double>(attitude.gyro[2]));

    const float* accel = sensor_.filteredAccel();
    if (accel == nullptr) {
        accel = sensor_.sample().accel;
    }
    const Eigen::Vector3d next_acceleration(
        static_cast<double>(accel[0] * acceleration_scale_),
        static_cast<double>(accel[1] * acceleration_scale_),
        static_cast<double>(accel[2] * acceleration_scale_));

    if (!next_angular_velocity.allFinite() ||
        !next_acceleration.allFinite()) {
        lock_memory();
        state_valid_ = false;
        unlock_memory();
        return false;
    }

    lock_memory();
    orientation = next_orientation_normalized;
    angular_velocity = next_angular_velocity;
    acceleration = next_acceleration;
    state_valid_ = true;
    unlock_memory();
    return true;
}
