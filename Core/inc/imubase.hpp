#pragma once

#include <Eigen/Dense>

#include <atomic>
#include <cstdint>

class IMUBase {
public:
    IMUBase()
        : orientation_(Eigen::Quaternionf::Identity()),
          angular_velocity_(Eigen::Vector3d::Zero()),
          acceleration_(Eigen::Vector3d::Zero())
    {
    }
    virtual ~IMUBase() = default;

    virtual void init() { return; }
    virtual bool is_ready() { return true; }
    virtual bool update(Eigen::Quaternionf& q,
                        Eigen::Vector3d& angular,
                        Eigen::Vector3d& acc,
                        const float& dt)
    {
        (void)dt;
        q = orientation_;
        angular = angular_velocity_;
        acc = acceleration_;
        return true;
    }

    bool read_state(Eigen::Quaternionf& q,
                    Eigen::Vector3d& angular,
                    Eigen::Vector3d& acc) const
    {
        for (;;) {
            const uint32_t sequence_begin =
                state_sequence_.load(std::memory_order_acquire);
            if ((sequence_begin & 1U) != 0U) {
                continue;
            }

            q = orientation_;
            angular = angular_velocity_;
            acc = acceleration_;
            const bool valid = state_valid_;

            const uint32_t sequence_end =
                state_sequence_.load(std::memory_order_acquire);
            if (sequence_begin == sequence_end) {
                return valid;
            }
        }
    }

protected:
    void publish_state(const Eigen::Quaternionf& q,
                       const Eigen::Vector3d& angular,
                       const Eigen::Vector3d& acc)
    {
        state_sequence_.fetch_add(1U, std::memory_order_acq_rel);
        orientation_ = q;
        angular_velocity_ = angular;
        acceleration_ = acc;
        state_valid_ = true;
        state_sequence_.fetch_add(1U, std::memory_order_release);
    }

private:
    Eigen::Quaternionf orientation_;
    Eigen::Vector3d angular_velocity_;
    Eigen::Vector3d acceleration_;
    bool state_valid_ = true;
    mutable std::atomic<uint32_t> state_sequence_{0U};
};
