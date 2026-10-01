#pragma once

#include "dmbase.hpp"

struct DM4310MotorParam : public DMMotorParam {
    DM4310MotorParam()
    {
        position_limit = 12.5F;
        velocity_limit = 45.0F;
        torque_limit = 10.0F;
        reduction_ratio = 1.0F;
    }
};

using DM4310MotorState = DMMotorState;

class DM4310Motor final : public DMMotorBase {
public:
    using Error = DMMotorBase::Error;

    explicit DM4310Motor(void* param = nullptr);
    explicit DM4310Motor(const DM4310MotorParam& param);
    DM4310Motor(bsp::FdcanBus& bus, const DM4310MotorParam& param);
};
