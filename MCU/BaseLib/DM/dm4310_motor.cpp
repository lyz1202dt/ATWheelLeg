#include "dm4310_motor.hpp"

DM4310Motor::DM4310Motor(void* param) : DMMotorBase()
{
    if (param != nullptr) {
        initialize(*static_cast<DM4310MotorParam*>(param));
    }
}

DM4310Motor::DM4310Motor(const DM4310MotorParam& param) : DMMotorBase(param) {}

DM4310Motor::DM4310Motor(bsp::FdcanBus& bus, const DM4310MotorParam& param) : DMMotorBase(bus, param) {}
