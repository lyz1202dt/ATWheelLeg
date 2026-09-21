#include "m3508.hpp"

M3508Motor::M3508Motor(void* param) : Motor(param)
{
    if (param != nullptr) {
        initialize(*static_cast<M3508MotorParam*>(param));
    }
}

M3508Motor::M3508Motor(const M3508MotorParam& param) : Motor(nullptr)
{
    initialize(param);
}

M3508Motor::M3508Motor(bsp::FdcanBus& bus, const M3508MotorParam& param)
    : Motor(nullptr)
{
    M3508MotorParam bus_param = param;
    bus_param.bus = &bus;
    initialize(bus_param);
}

M3508Motor::M3508Motor(bsp::FdcanBus& bus, uint8_t motor_id)
    : Motor(nullptr)
{
    M3508MotorParam param = {};
    param.bus = &bus;
    param.motor_id = motor_id;
    initialize(param);
}

void M3508Motor::initialize(const M3508MotorParam& param)
{
    if (!is_valid_motor_id(param.motor_id) || param.current_limit <= 0 || param.reduction_ratio == 0.0F) {
        return;
    }

    const uint32_t derived_tx_id = command_id_from_motor_id(param.motor_id);
    const uint32_t derived_rx_id = feedback_id_from_motor_id(param.motor_id);
    const uint32_t tx_id = param.tx_id == 0U ? derived_tx_id : param.tx_id;
    const uint32_t rx_id = param.rx_id == 0U ? derived_rx_id : param.rx_id;

    if ((tx_id != kLowCommandId && tx_id != kHighCommandId) || rx_id != derived_rx_id) {
        return;
    }

    bus_ = param.bus;
    motor_id_ = param.motor_id;
    tx_id_ = tx_id;
    rx_id_ = rx_id;
    current_limit_ = param.current_limit;
    reduction_ratio_ = param.reduction_ratio;

    if (param.auto_register_feedback && bus_ != nullptr) {
        register_feedback_callback();
    }
}

bool M3508Motor::init()
{
    return is_valid_motor_id(motor_id_) && tx_id_ != 0U && rx_id_ != 0U
           && current_limit_ > 0 && reduction_ratio_ != 0.0F;
}

bool M3508Motor::enable()
{
    return init();
}

bool M3508Motor::disable()
{
    command_current_ = 0;
    return true;
}

int M3508Motor::has_error()
{
    // RM3508 reports no separate error code in its standard 0x201..0x208 frame.
    return 0;
}

bool M3508Motor::clear_error(int)
{
    return true;
}

bool M3508Motor::set_command(float, float, float torque, float, float)
{
    // The common Motor API expresses torque at the gearbox output in N.m.
    const float rotor_torque = output_to_motor_torque(torque);
    return set_current_command(rotor_torque / kRotorTorqueConstant);
}

bool M3508Motor::read_state()
{
    return state_.ready;
}

bool M3508Motor::handle_feedback(const bsp::FdcanBus::Frame& frame)
{
    const uint8_t length = bsp::FdcanBus::dlcToLength(frame.data_length);
    const uint8_t* data = frame.data;
    if (data == nullptr || length < kFeedbackLength || frame.id != rx_id_) {
        return false;
    }

    const uint16_t mechanical_angle =
        static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8U) | static_cast<uint16_t>(data[1]));
    const int16_t speed_rpm =
        static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8U) | static_cast<uint16_t>(data[3]));
    const int16_t raw_current =
        static_cast<int16_t>((static_cast<uint16_t>(data[4]) << 8U) | static_cast<uint16_t>(data[5]));

    state_.mechanical_angle = mechanical_angle;
    state_.speed_rpm = speed_rpm;
    state_.raw_current = raw_current;
    state_.current = raw_current_to_current(raw_current);
    state_.rotor_torque = state_.current * kRotorTorqueConstant;
    state_.output_torque = motor_to_output_torque(state_.rotor_torque);
    state_.temperature = data[6];

    if (!state_.ready) {
        state_.last_angle = mechanical_angle;
        state_.round = 0;
        state_.angle = mechanical_angle;
        state_.offset = state_.angle;
        state_.ready = true;
    } else {
        const int16_t diff = static_cast<int16_t>(mechanical_angle - state_.last_angle);
        if (diff > 4000) {
            --state_.round;
        } else if (diff < -4000) {
            ++state_.round;
        }
        state_.angle = static_cast<int32_t>(state_.round) * 8192 + mechanical_angle;
        state_.last_angle = mechanical_angle;
    }

    state_.angle_deg = static_cast<float>(state_.angle) * kEncoderToDeg;
    state_.actual_position = state_.angle - state_.offset;
    update_base_state();
    return true;
}

bool M3508Motor::is_valid_motor_id(uint8_t motor_id)
{
    return motor_id >= 1U && motor_id <= kMaxMotorId;
}

uint32_t M3508Motor::feedback_id_from_motor_id(uint8_t motor_id)
{
    return kFeedbackBaseId + motor_id;
}

uint32_t M3508Motor::command_id_from_motor_id(uint8_t motor_id)
{
    return motor_id <= 4U ? kLowCommandId : kHighCommandId;
}

int16_t M3508Motor::clamp_raw_current(float raw_current, int16_t limit)
{
    if (raw_current != raw_current) {
        return 0;
    }
    if (raw_current >= static_cast<float>(limit)) {
        return limit;
    }
    if (raw_current <= -static_cast<float>(limit)) {
        return static_cast<int16_t>(-limit);
    }
    return static_cast<int16_t>(raw_current);
}

float M3508Motor::raw_current_to_current(int16_t raw_current)
{
    return static_cast<float>(raw_current) * kRawCurrentToAmp;
}

int16_t M3508Motor::current_to_raw_current(float current, int16_t limit)
{
    return clamp_raw_current(current * kAmpToRawCurrent, limit);
}

bool M3508Motor::register_feedback_callback()
{
    if (bus_ == nullptr) {
        return false;
    }
    if (feedback_callback_registered_) {
        return true;
    }

    const HAL_StatusTypeDef status =
        bus_->register_recv_cb([this](const bsp::FdcanBus::Frame& frame) {
            handle_feedback(frame);
        });
    feedback_callback_registered_ = (status == HAL_OK);
    return feedback_callback_registered_;
}

uint8_t M3508Motor::command_slot(uint32_t command_id) const
{
    if (command_id != tx_id_) {
        return 0xFFU;
    }
    return tx_id_ == kLowCommandId
               ? static_cast<uint8_t>(motor_id_ - 1U)
               : static_cast<uint8_t>(motor_id_ - 5U);
}

float M3508Motor::output_to_motor_torque(float output_torque) const
{
    // reduction_ratio_ is rotor angle / output angle, including direction.
    return output_torque / reduction_ratio_;
}

float M3508Motor::motor_to_output_torque(float motor_torque) const
{
    return motor_torque * reduction_ratio_;
}

void M3508Motor::update_base_state()
{
    state.r = state_.round;
    state.rad = static_cast<float>(state_.mechanical_angle) * kEncoderToRad / reduction_ratio_;
    state.continue_rad = static_cast<float>(state_.actual_position) * kEncoderToRad / reduction_ratio_;
    state.vel = static_cast<float>(state_.speed_rpm) * kRpmToRadPerSecond / reduction_ratio_;
    state.toqeue = state_.output_torque;
}

bool M3508Motor::set_current_command(float current)
{
    command_current_ = current_to_raw_current(current, current_limit_);
    return true;
}

bool M3508Motor::set_raw_current_command(int16_t raw_current)
{
    command_current_ = clamp_raw_current(static_cast<float>(raw_current), current_limit_);
    return true;
}

bool M3508Motor::send_command(bsp::FdcanBus& bus,
                              M3508Motor& motor1,
                              M3508Motor& motor2,
                              M3508Motor& motor3,
                              M3508Motor& motor4)
{
    M3508Motor* motors[] = {&motor1, &motor2, &motor3, &motor4};
    return send_commands(bus, motor1.tx_id_, motors, 4U);
}

bool M3508Motor::send_command(bsp::FdcanBus& bus,
                              M3508Motor& motor1,
                              M3508Motor& motor2,
                              M3508Motor& motor3)
{
    M3508Motor* motors[] = {&motor1, &motor2, &motor3};
    return send_commands(bus, motor1.tx_id_, motors, 3U);
}

bool M3508Motor::send_command(bsp::FdcanBus& bus, M3508Motor& motor1, M3508Motor& motor2)
{
    M3508Motor* motors[] = {&motor1, &motor2};
    return send_commands(bus, motor1.tx_id_, motors, 2U);
}

bool M3508Motor::send_command(bsp::FdcanBus& bus, M3508Motor& motor1)
{
    M3508Motor* motors[] = {&motor1};
    return send_commands(bus, motor1.tx_id_, motors, 1U);
}

bool M3508Motor::send_commands(bsp::FdcanBus& bus,
                               uint32_t command_id,
                               M3508Motor* const* motors,
                               uint8_t motor_count)
{
    if ((command_id != kLowCommandId && command_id != kHighCommandId) ||
        motors == nullptr || motor_count == 0U || motor_count > 4U) {
        return false;
    }

    uint8_t data[kCommandLength] = {};
    bool used_slots[4] = {};

    for (uint8_t i = 0U; i < motor_count; ++i) {
        if (motors[i] == nullptr || motors[i]->tx_id_ != command_id) {
            return false;
        }

        const uint8_t slot = motors[i]->command_slot(command_id);
        if (slot >= 4U || used_slots[slot]) {
            return false;
        }
        used_slots[slot] = true;

        const uint16_t value = static_cast<uint16_t>(motors[i]->command_current_);
        data[2U * slot] = static_cast<uint8_t>(value >> 8U);
        data[2U * slot + 1U] = static_cast<uint8_t>(value);
    }

    return bus.transmit(command_id, data, kCommandLength) == HAL_OK;
}
