#pragma once

#include "imubase.hpp"
#include "motorbase.hpp"
#include <Eigen/Dense>


class Controller {
public:
    Controller(IMUBase* imu, Motor* lf, Motor* rf, Motor* lb, Motor* rb, Motor* lw, Motor* rw);
    bool update(float dt);
    void input(float velocity,float omega,float height=0.21f,int mode=0);  //机器人整体的参考输入，期望前进的线速度和角速度,本体高度，控制模式0-默认
    IMUBase* imu;
    Motor *lf, rf, lb, rb, lw, rw;
};
