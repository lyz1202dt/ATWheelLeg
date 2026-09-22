#pragma once

#include "imubase.hpp"
#include "motorbase.hpp"


class ControllerBase {
public:
    ControllerBase(IMUBase* imu, Motor* lf, Motor* rf, Motor* lb, Motor* rb, Motor* lw, Motor* rw);
    virtual ~ControllerBase() = default;

    virtual bool update(float dt);
    virtual void input(float velocity, float omega, float height, int mode);
    IMUBase* imu;
    Motor* lf;
    Motor* rf;
    Motor* lb;
    Motor* rb;
    Motor* lw;
    Motor* rw;
private:

};
