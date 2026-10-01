#pragma once

#include "fdcan.hpp"
#include "motorbase.hpp"

#include <cstdint>

struct DMMotorParam {
    bsp::FdcanBus* bus = nullptr;
    uint32_t tx_id = 0U;
    // Feedback CAN ID, called Master ID in the DAMIAO manuals.
    uint32_t rx_id = 0U;

    // These limits must match the P_MAX, V_MAX and T_MAX configured in the
    // DAMIAO assistant for MIT mode.
    float position_limit = 12.5F;
    float velocity_limit = 45.0F;
    float torque_limit = 10.0F;

    // CAN protocol values are treated as motor-side values. Program-facing
    // Motor::state values are output/joint-side:
    // motor_position = joint_position * reduction_ratio + zero_angle_offset.
    // Use a negative reduction_ratio when the output direction is reversed.
    float zero_angle_offset = 0.0F;
    float reduction_ratio = 1.0F;

    bool auto_register_feedback = true;
};

struct DMMotorState {
    uint8_t id = 0U;
    uint8_t error = 0U;

    // Raw CAN feedback after protocol scaling.
    float motor_position = 0.0F;
    float motor_continuous_position = 0.0F;
    float motor_velocity = 0.0F;
    float motor_torque = 0.0F;

    // Program-facing output-shaft state after applying reduction/offset.
    float joint_position = 0.0F;
    float joint_continuous_position = 0.0F;
    float joint_velocity = 0.0F;
    float joint_torque = 0.0F;

    float mos_temperature = 0.0F;
    float rotor_temperature = 0.0F;
};

class DMMotorBase : public Motor {
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

    DMMotorBase();
    explicit DMMotorBase(const DMMotorParam& param);
    DMMotorBase(bsp::FdcanBus& bus, const DMMotorParam& param);

    bool init() override;
    bool enable() override;
    bool disable() override;
    int has_error() override;
    bool clear_error(int cmd = 0) override;
    bool read_state() override;
    bool set_command(float pos, float vel, float torque, float kp, float kd) override;

    void set_offset(float offset);
    void set_ratio(float ratio);
    void reset_continuous_position();

    bool mit_control(float joint_pos, float joint_vel, float joint_torque, float joint_kp, float joint_kd);
    bool mit_control_raw(float motor_pos, float motor_vel, float motor_torque, float kp, float kd);
    bool position_velocity_control(float joint_pos, float joint_vel);
    bool velocity_control(float joint_vel);

    bool handle_feedback(const bsp::FdcanBus::Frame& frame);

    const DMMotorState& dm_state() const;

protected:
    void initialize(const DMMotorParam& param);

private:
    static constexpr uint8_t kCommandLength = 8U;
    static constexpr uint8_t kVelocityCommandLength = 4U;
    static constexpr uint8_t kFeedbackLength = 8U;
    static constexpr uint8_t kFeedbackIdMask = 0x0FU;
    static constexpr uint32_t kPositionVelocityIdOffset = 0x100U;
    static constexpr uint32_t kVelocityIdOffset = 0x200U;
    static constexpr float kMitKpLimit = 500.0F;
    static constexpr float kMitKdLimit = 5.0F;

    float motor_to_joint_position(float motor_position) const;
    float motor_to_joint_velocity(float motor_velocity) const;
    float motor_to_joint_torque(float motor_torque) const;
    float joint_to_motor_position(float joint_position) const;
    float joint_to_motor_velocity(float joint_velocity) const;
    float joint_to_motor_torque(float joint_torque) const;

    static float clamp(float value, float min_value, float max_value);
    static uint32_t float_to_uint(float value, float min_value, float max_value, uint8_t bits);
    static float uint_to_float(uint32_t value, float min_value, float max_value, uint8_t bits);
    static void pack_float_le(float value, uint8_t* data);

    bool send(uint32_t id, const uint8_t* data, uint8_t length);
    bool send_special_command(uint8_t command);
    bool register_feedback_callback();
    void update_continuous_position(float motor_position);
    void update_joint_state();

    bsp::FdcanBus* bus_ = nullptr;
    uint32_t tx_id_ = 0U;
    uint32_t rx_id_ = 0U;

    float position_limit_ = 12.5F;
    float velocity_limit_ = 45.0F;
    float torque_limit_ = 10.0F;

    float zero_angle_offset_ = 0.0F;
    float reduction_ratio_ = 1.0F;

    DMMotorState state_ = {};
    bool has_state_ = false;
    bool feedback_callback_registered_ = false;
    bool continuous_position_ready_ = false;
    float last_motor_position_ = 0.0F;
};
