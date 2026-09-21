#pragma once

#include "estimaterbase.hpp"


class WheelLeggedEstimater : public EstimaterBase<10, 10> {
public:
    enum RobotState {
        TOUCH,
        FLY
    };

    WheelLeggedEstimater();

    // 状态更新
    bool update(InputVector& measure, OutputVector& output) override;

    // 复位估计器
    bool reset(OutputVector& output) override;

    // 机器人处于不同的状态，对应的状态估计器也不一样
    bool set_mode(RobotState state);
};
