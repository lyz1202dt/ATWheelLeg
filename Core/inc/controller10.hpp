#pragma once

//10状态，WBC全身LQR建模控制方案

#include "controllerbase.hpp"

class Controller10 : public ControllerBase {
public:
    Controller10(IMUBase* imu, Motor* lf, Motor* rf, Motor* lb, Motor* rb, Motor* lw, Motor* rw);
};