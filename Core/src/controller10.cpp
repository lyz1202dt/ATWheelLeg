#include "controller10.hpp"

Controller10::Controller10(IMUBase* imu, Motor* lf, Motor* rf, Motor* lb, Motor* rb, Motor* lw, Motor* rw)
    : ControllerBase(imu, lf, rf, lb, rb, lw, rw) {

    }


bool Controller10::update(float dt) {
    Eigen::Vector<double,4> u;
    Eigen::Vector<double,10> x;

    if(state==IDEL)
    {

    }
}

void Controller10::input(float velocity, float omega, float height, int mode) {}
