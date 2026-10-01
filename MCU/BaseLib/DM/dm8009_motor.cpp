#include "dm8009_motor.hpp"

DM8009Motor::DM8009Motor(void* param) : DMMotorBase()
{
    if (param != nullptr) {
        initialize(*static_cast<DM8009MotorParam*>(param));
    }
}

DM8009Motor::DM8009Motor(const DM8009MotorParam& param) : DMMotorBase(param) {}

DM8009Motor::DM8009Motor(bsp::FdcanBus& bus, const DM8009MotorParam& param) : DMMotorBase(bus, param) {}
