#include "controllerbase.hpp"

ControllerBase::ControllerBase(IMUBase* imu_in,
                               Motor* lf_in,
                               Motor* rf_in,
                               Motor* lb_in,
                               Motor* rb_in,
                               Motor* lw_in,
                               Motor* rw_in)
    : imu(imu_in)
    , lf(lf_in)
    , rf(rf_in)
    , lb(lb_in)
    , rb(rb_in)
    , lw(lw_in)
    , rw(rw_in) {}

bool ControllerBase::update(float dt)
{
    (void)dt;
    return true;
}

void ControllerBase::input(float velocity, float omega, float height, int mode)
{
    (void)velocity;
    (void)omega;
    (void)height;
    (void)mode;
}
