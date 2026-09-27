#pragma once

#include "imubase.hpp"
#include "motorbase.hpp"

#include <cstdio>
#include <cstdint>
#include <functional>
#include <utility>


class ControllerBase {
public:
    using DebugLogger = std::function<void(const char*)>;

    ControllerBase(IMUBase* imu,
                   Motor* lf,
                   Motor* rf,
                   Motor* lb,
                   Motor* rb,
                   Motor* lw,
                   Motor* rw,
                   DebugLogger debug_logger = nullptr);
    virtual ~ControllerBase() = default;

    virtual bool update(uint64_t ms);
    virtual void input(float velocity, float omega, float height, int mode);
    void set_debug_logger(DebugLogger logger);

protected:
    template <typename... Args>
    void debug_log(const char* format, Args... args) const {
        if (!debug_logger_ || format == nullptr) {
            return;
        }

        char buffer[256]{};
        const int written = std::snprintf(buffer, sizeof(buffer), format, args...);
        if (written < 0) {
            return;
        }
        buffer[sizeof(buffer) - 1U] = '\0';
        debug_logger_(buffer);
    }

public:
    IMUBase* imu;
    Motor* lf;
    Motor* rf;
    Motor* lb;
    Motor* rb;
    Motor* lw;
    Motor* rw;
private:
    DebugLogger debug_logger_;
};
