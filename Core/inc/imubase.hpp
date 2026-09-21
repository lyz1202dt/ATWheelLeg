#pragma once

#include <Eigen/Dense>

#include <atomic>
#include <cstdint>

class IMUBase {
public:
    IMUBase()
        : orientation(Eigen::Quaternionf::Identity())
        , angular_velocity(Eigen::Vector3d::Zero())
        , acceleration(Eigen::Vector3d::Zero()) {}
    virtual ~IMUBase() = default;

    virtual void init() { return; }
    virtual bool is_ready() { return true; }
    virtual bool update(const float& dt) {
        (void)dt;
        return true;
    }

    void lock_memory();
    void unlock_memory();

    Eigen::Quaternionf orientation;
    Eigen::Vector3d angular_velocity;
    Eigen::Vector3d acceleration;
    bool state_valid_ = true;
    mutable std::atomic<uint32_t> state_sequence_{0U};
};

inline void IMUBase::lock_memory()
{
    uint32_t sequence = state_sequence_.load(std::memory_order_relaxed);
    for (;;) {
        if ((sequence & 1U) != 0U) {
            sequence = state_sequence_.load(std::memory_order_relaxed);
            continue;
        }

        if (state_sequence_.compare_exchange_weak(
                sequence,
                sequence + 1U,
                std::memory_order_acquire,
                std::memory_order_relaxed)) {
            return;
        }
    }
}

inline void IMUBase::unlock_memory()
{
    state_sequence_.fetch_add(1U, std::memory_order_release);
}
