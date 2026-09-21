#include "dm4310_motor.hpp"

static_assert(sizeof(float) == 4U, "DM4310 float control frames require 32-bit float");

DM4310Motor::DM4310Motor(void* param) : Motor(param)
{
    if (param != nullptr) {
        configure(*static_cast<DM4310MotorParam*>(param));
    }
}

DM4310Motor::DM4310Motor(const DM4310MotorParam& param) : Motor(nullptr)
{
    configure(param);
}

bool DM4310Motor::init()
{
    return bus_ != nullptr && reduction_ratio_ != 0.0F && position_limit_ > 0.0F
           && velocity_limit_ > 0.0F && torque_limit_ > 0.0F;
}

bool DM4310Motor::set_command(float pos, float vel, float torque, float kp, float kd)
{
    return mit_control(pos, vel, torque, kp, kd);
}

bool DM4310Motor::configure(const DM4310MotorParam& param)
{
    if (param.reduction_ratio == 0.0F || param.position_limit <= 0.0F || param.velocity_limit <= 0.0F
        || param.torque_limit <= 0.0F) {
        return false;
    }

    bus_ = param.bus;
    tx_id_ = param.tx_id;
    rx_id_ = param.rx_id;

    position_limit_ = param.position_limit;
    velocity_limit_ = param.velocity_limit;
    torque_limit_ = param.torque_limit;

    zero_angle_offset_ = param.zero_angle_offset;
    reduction_ratio_ = param.reduction_ratio;
    update_joint_state();
    return true;
}

bool DM4310Motor::set_encoder_to_joint(float zero_angle_offset, float reduction_ratio)
{
    if (reduction_ratio == 0.0F) {
        return false;
    }

    zero_angle_offset_ = zero_angle_offset;
    reduction_ratio_ = reduction_ratio;
    update_joint_state();
    return true;
}

bool DM4310Motor::mit_control(float joint_pos, float joint_vel, float joint_torque, float joint_kp, float joint_kd)
{
    const float motor_pos = joint_to_motor_position(joint_pos);
    const float motor_vel = joint_to_motor_velocity(joint_vel);
    const float motor_torque = joint_to_motor_torque(joint_torque);
    const float ratio_squared = reduction_ratio_ * reduction_ratio_;
    const float motor_kp = joint_kp / ratio_squared;
    const float motor_kd = joint_kd / ratio_squared;

    return mit_control_raw(motor_pos, motor_vel, motor_torque, motor_kp, motor_kd);
}

bool DM4310Motor::mit_control_raw(float motor_pos, float motor_vel, float motor_torque, float kp, float kd)
{
    const uint32_t pos_tmp = float_to_uint(motor_pos, -position_limit_, position_limit_, 16U);
    const uint32_t vel_tmp = float_to_uint(motor_vel, -velocity_limit_, velocity_limit_, 12U);
    const uint32_t torque_tmp = float_to_uint(motor_torque, -torque_limit_, torque_limit_, 12U);
    const uint32_t kp_tmp = float_to_uint(kp, 0.0F, 500.0F, 12U);
    const uint32_t kd_tmp = float_to_uint(kd, 0.0F, 5.0F, 12U);

    uint8_t data[kCommandLength] = {};
    data[0] = static_cast<uint8_t>(pos_tmp >> 8U);
    data[1] = static_cast<uint8_t>(pos_tmp);
    data[2] = static_cast<uint8_t>(vel_tmp >> 4U);
    data[3] = static_cast<uint8_t>(((vel_tmp & 0x0FU) << 4U) | (kp_tmp >> 8U));
    data[4] = static_cast<uint8_t>(kp_tmp);
    data[5] = static_cast<uint8_t>(kd_tmp >> 4U);
    data[6] = static_cast<uint8_t>(((kd_tmp & 0x0FU) << 4U) | (torque_tmp >> 8U));
    data[7] = static_cast<uint8_t>(torque_tmp);

    return send(tx_id_, data, kCommandLength);
}

bool DM4310Motor::enable()
{
    uint8_t data[kCommandLength] = {};
    for (uint8_t i = 0U; i < kCommandLength - 1U; ++i) {
        data[i] = 0xFFU;
    }
    data[7] = 0xFCU;
    return send(tx_id_, data, kCommandLength);
}

bool DM4310Motor::disable()
{
    uint8_t data[kCommandLength] = {};
    for (uint8_t i = 0U; i < kCommandLength - 1U; ++i) {
        data[i] = 0xFFU;
    }
    data[7] = 0xFDU;
    return send(tx_id_, data, kCommandLength);
}

int DM4310Motor::has_error()
{
    if (!has_state_) {
        return 0;
    }

    return state_.error;
}

bool DM4310Motor::clear_error(int)
{
    uint8_t data[kCommandLength] = {};
    for (uint8_t i = 0U; i < kCommandLength - 1U; ++i) {
        data[i] = 0xFFU;
    }
    data[7] = 0xFBU;
    return send(tx_id_, data, kCommandLength);
}

bool DM4310Motor::handle_feedback(const bsp::FdcanBus::Frame& frame)
{
    const uint8_t length = bsp::FdcanBus::dlcToLength(frame.data_length);
    return handle_feedback(frame.id, frame.data, length);
}

bool DM4310Motor::handle_feedback(uint32_t frame_id, const uint8_t* data, uint8_t length)
{
    if (data == nullptr || length < kFeedbackLength || frame_id != rx_id_) {
        return false;
    }

    const uint8_t feedback_id = data[0] & kFeedbackIdMask;
    if (feedback_id != (tx_id_ & kFeedbackIdMask)) {
        return false;
    }

    const uint32_t pos_tmp = (static_cast<uint32_t>(data[1]) << 8U) | static_cast<uint32_t>(data[2]);
    const uint32_t vel_tmp = (static_cast<uint32_t>(data[3]) << 4U) | (static_cast<uint32_t>(data[4]) >> 4U);
    const uint32_t torque_tmp = ((static_cast<uint32_t>(data[4]) & 0x0FU) << 8U) | static_cast<uint32_t>(data[5]);

    state_.id = feedback_id;
    state_.error = data[0] >> 4U;
    state_.motor_position = uint_to_float(pos_tmp, -position_limit_, position_limit_, 16U);
    state_.motor_velocity = uint_to_float(vel_tmp, -velocity_limit_, velocity_limit_, 12U);
    state_.motor_torque = uint_to_float(torque_tmp, -torque_limit_, torque_limit_, 12U);
    state_.mos_temperature = static_cast<float>(data[6]);
    state_.rotor_temperature = static_cast<float>(data[7]);
    update_joint_state();
    has_state_ = true;
    return true;
}

float DM4310Motor::motor_to_joint_position(float motor_position) const
{
    // Inverse of: motor_position = joint_position * reduction_ratio_ + zero_angle_offset_.
    return (motor_position - zero_angle_offset_) / reduction_ratio_;
}

void DM4310Motor::set_offset(float offset)
{
    zero_angle_offset_ = offset;
    update_joint_state();
}

void DM4310Motor::set_ratio(float ratio)
{
    if (ratio == 0.0F) {
        return;
    }

    reduction_ratio_ = ratio;
    update_joint_state();
}

float DM4310Motor::motor_to_joint_velocity(float motor_velocity) const
{
    return motor_velocity / reduction_ratio_;
}

float DM4310Motor::motor_to_joint_torque(float motor_torque) const
{
    return motor_torque * reduction_ratio_;
}

float DM4310Motor::joint_to_motor_position(float joint_position) const
{
    // Convert the program-facing output-shaft position to the CAN rotor position.
    return joint_position * reduction_ratio_ + zero_angle_offset_;
}

float DM4310Motor::joint_to_motor_velocity(float joint_velocity) const
{
    return joint_velocity * reduction_ratio_;
}

float DM4310Motor::joint_to_motor_torque(float joint_torque) const
{
    return joint_torque / reduction_ratio_;
}

float DM4310Motor::clamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

uint32_t DM4310Motor::float_to_uint(float value, float min_value, float max_value, uint8_t bits)
{
    const float clamped = clamp(value, min_value, max_value);
    const float span = max_value - min_value;
    const uint32_t scale = (1UL << bits) - 1UL;
    return static_cast<uint32_t>((clamped - min_value) * static_cast<float>(scale) / span);
}

float DM4310Motor::uint_to_float(uint32_t value, float min_value, float max_value, uint8_t bits)
{
    const float span = max_value - min_value;
    const uint32_t scale = (1UL << bits) - 1UL;
    return static_cast<float>(value) * span / static_cast<float>(scale) + min_value;
}

bool DM4310Motor::send(uint32_t id, const uint8_t* data, uint8_t length)
{
    if (bus_ == nullptr) {
        return false;
    }

    return bus_->transmit(id, data, length) == HAL_OK;
}

void DM4310Motor::update_joint_state()
{
    if (reduction_ratio_ == 0.0F) {
        return;
    }

    state_.joint_position = motor_to_joint_position(state_.motor_position);
    state_.joint_velocity = motor_to_joint_velocity(state_.motor_velocity);
    state_.joint_torque = motor_to_joint_torque(state_.motor_torque);
}
