#pragma once

#include "controllerbase.hpp"
#include "wheel_leg_estimater.hpp"

class RobotVMCLQRController : public ControllerBase<10, 6> {
public:
    explicit RobotVMCLQRController(WheelLeggedEstimater &estimater);
    bool update(InputVector& measure, OutputVector& output) override;
    bool reset() override;
    WheelLeggedEstimater &estimater;
};
