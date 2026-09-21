#pragma once

#include "fdcan.hpp"
#include "motorbase.hpp"

#include <cstdint>

struct DM4310MotorParam {
    bsp::FdcanBus* bus = nullptr;
    uint32_t tx_id = 0U;
    // Feedback CAN ID, called Master ID in the DM4310 manual.
    uint32_t rx_id = 0U;

    float position_limit = 12.5F;
    float velocity_limit = 45.0F;
    float torque_limit = 10.0F;

    // CAN feedback is rotor-side position. Program-facing position is output-side:
    // rotor_position = output_position * reduction_ratio + zero_angle_offset.
    // Use a negative reduction_ratio when the output direction is reversed.
    float zero_angle_offset = 0.0F;
    float reduction_ratio = 1.0F;
};

struct DM4310MotorState {
    uint8_t id = 0U;
    uint8_t error = 0U;

    // Raw CAN feedback from the motor rotor side.
    float motor_position = 0.0F;
    float motor_velocity = 0.0F;
    float motor_torque = 0.0F;

    // Program-facing output-shaft state after applying the mapping above.
    float joint_position = 0.0F;
    float joint_velocity = 0.0F;
    float joint_torque = 0.0F;

    float mos_temperature = 0.0F;
    float rotor_temperature = 0.0F;
};

class DM4310Motor : public Motor {
public:
    enum class Error : uint8_t {
        None = 0x0U,
        OverVoltage = 0x8U,
        UnderVoltage = 0x9U,
        OverCurrent = 0xAU,
        MosOverTemperature = 0xBU,
        RotorOverTemperature = 0xCU,
        CommunicationLost = 0xDU,
        Overload = 0xEU,
    };

    explicit DM4310Motor(void* param);
    explicit DM4310Motor(const DM4310MotorParam& param);

    bool init() override;
    bool enable() override;
    bool disable() override;
    int has_error() override;
    bool clear_error(int cmd = 0) override;
    bool set_command(float pos, float vel, float torque, float kp, float kd) override;

    void set_offset(float offset);
    void set_ratio(float ratio);

    bool configure(const DM4310MotorParam& param);
    bool set_encoder_to_joint(float zero_angle_offset, float reduction_ratio);

    bool mit_control(float joint_pos, float joint_vel, float joint_torque, float joint_kp, float joint_kd);
    bool mit_control_raw(float motor_pos, float motor_vel, float motor_torque, float kp, float kd);

    bool handle_feedback(const bsp::FdcanBus::Frame& frame);
    bool handle_feedback(uint32_t frame_id, const uint8_t* data, uint8_t length);

    const DM4310MotorState& state() const { return state_; }
    bool has_state() const { return has_state_; }

    float motor_to_joint_position(float motor_position) const;
    float motor_to_joint_velocity(float motor_velocity) const;
    float motor_to_joint_torque(float motor_torque) const;
    float joint_to_motor_position(float joint_position) const;
    float joint_to_motor_velocity(float joint_velocity) const;
    float joint_to_motor_torque(float joint_torque) const;

private:
    static constexpr uint8_t kCommandLength = 8U;
    static constexpr uint8_t kFeedbackLength = 8U;
    static constexpr uint8_t kFeedbackIdMask = 0x0FU;

    static float clamp(float value, float min_value, float max_value);
    static uint32_t float_to_uint(float value, float min_value, float max_value, uint8_t bits);
    static float uint_to_float(uint32_t value, float min_value, float max_value, uint8_t bits);

    bool send(uint32_t id, const uint8_t* data, uint8_t length);
    void update_joint_state();

    bsp::FdcanBus* bus_ = nullptr;
    uint32_t tx_id_ = 0U;
    uint32_t rx_id_ = 0U;

    float position_limit_ = 12.5F;
    float velocity_limit_ = 45.0F;
    float torque_limit_ = 10.0F;

    float zero_angle_offset_ = 0.0F;
    float reduction_ratio_ = 1.0F;

    DM4310MotorState state_ = {};
    bool has_state_ = false;
};
