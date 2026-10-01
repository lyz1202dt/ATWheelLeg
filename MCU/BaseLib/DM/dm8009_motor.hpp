#pragma once

#include "dmbase.hpp"

struct DM8009MotorParam : public DMMotorParam {
    DM8009MotorParam()
    {
        position_limit = 12.5F;
        velocity_limit = 45.0F;
        torque_limit = 40.0F;
        reduction_ratio = 9.0F;
    }
};

using DM8009MotorState = DMMotorState;

class DM8009Motor final : public DMMotorBase {
public:
    using Error = DMMotorBase::Error;

    explicit DM8009Motor(void* param = nullptr);
    explicit DM8009Motor(const DM8009MotorParam& param);
    DM8009Motor(bsp::FdcanBus& bus, const DM8009MotorParam& param);
};
