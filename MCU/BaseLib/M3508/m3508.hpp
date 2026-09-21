#pragma once

#include "fdcan.hpp"
#include "motorbase.hpp"

#include <cstdint>

struct M3508MotorParam {
    bsp::FdcanBus* bus = nullptr;

    // DJI/RM motor id on the CAN bus, valid range: 1..8.
    uint8_t motor_id = 1U;

    // Optional explicit CAN ids. Leave as 0 to derive from motor_id.
    // motor_id 1..4 use command id 0x200, 5..8 use command id 0x1FF.
    uint32_t tx_id = 0U;
    uint32_t rx_id = 0U;

    // CAN command raw current limit. The DJI C620 protocol maps +/-16384 to
    // +/-20 A phase current for RM3508.
    int16_t current_limit = 16384;
    bool auto_register_feedback = true;

    // Rotor-to-output reduction ratio. The standard M3508 gearbox is 3591/187.
    // Use a negative value when the output direction is reversed.
    float reduction_ratio = 3591.0F / 187.0F;
};

struct M3508MotorState {
    // Feedback values on the rotor side unless stated otherwise.
    uint16_t mechanical_angle = 0U;
    int16_t speed_rpm = 0;
    // Raw signed CAN value and phase current in A. +/-16384 <=> +/-20 A.
    int16_t raw_current = 0;
    float current = 0.0F;
    // Rotor torque and gearbox output torque in N.m.
    float rotor_torque = 0.0F;
    float output_torque = 0.0F;
    uint8_t temperature = 0U;

    uint16_t last_angle = 0U;
    int16_t round = 0;
    int32_t angle = 0;
    float angle_deg = 0.0F;

    int32_t offset = 0;
    int32_t actual_position = 0;
    bool ready = false;
};

class M3508Motor : public Motor {
public:
    static constexpr uint32_t kLowCommandId = 0x200U;
    static constexpr uint32_t kHighCommandId = 0x1FFU;
    static constexpr uint32_t kFeedbackBaseId = 0x200U;
    static constexpr uint8_t kMaxMotorId = 8U;
    static constexpr int16_t kDefaultCurrentLimit = 16384;
    static constexpr float kDefaultReductionRatio = 3591.0F / 187.0F;

    explicit M3508Motor(void* param = nullptr);
    explicit M3508Motor(const M3508MotorParam& param);
    M3508Motor(bsp::FdcanBus& bus, const M3508MotorParam& param);
    M3508Motor(bsp::FdcanBus& bus, uint8_t motor_id);

    bool init() override;
    bool enable() override;
    bool disable() override;
    int has_error() override;
    bool clear_error(int cmd = 0) override;
    bool set_command(float pos, float vel, float torque, float kp, float kd) override;
    bool read_state() override;

    bool handle_feedback(const bsp::FdcanBus::Frame& frame);
    bool set_current_command(float current);
    bool set_raw_current_command(int16_t raw_current);

    static bool send_command(bsp::FdcanBus& bus,
                             M3508Motor& motor1,
                             M3508Motor& motor2,
                             M3508Motor& motor3,
                             M3508Motor& motor4);
    static bool send_command(bsp::FdcanBus& bus, M3508Motor& motor1, M3508Motor& motor2, M3508Motor& motor3);
    static bool send_command(bsp::FdcanBus& bus, M3508Motor& motor1, M3508Motor& motor2);
    static bool send_command(bsp::FdcanBus& bus, M3508Motor& motor1);
    static bool send_commands(bsp::FdcanBus& bus,
                              uint32_t command_id,
                              M3508Motor* const* motors,
                              uint8_t motor_count);

private:
    static constexpr uint8_t kCommandLength = 8U;
    static constexpr uint8_t kFeedbackLength = 7U;
    static constexpr float kEncoderToDeg = 360.0F / 8192.0F;
    static constexpr float kEncoderToRad = 6.28318530717958647692F / 8192.0F;
    static constexpr float kRpmToRadPerSecond = 6.28318530717958647692F / 60.0F;
    static constexpr float kMaxProtocolCurrent = 20.0F;
    // The datasheet's 0.3 N.m/A is treated as the stock 3591/187 gearbox
    // output torque constant. Alternate gearboxes scale it by their ratio.
    static constexpr float kDefaultOutputTorqueConstant = 0.3F;
    static constexpr float kRotorTorqueConstant = kDefaultOutputTorqueConstant / kDefaultReductionRatio;
    static constexpr float kRawCurrentToAmp = kMaxProtocolCurrent / static_cast<float>(kDefaultCurrentLimit);
    static constexpr float kAmpToRawCurrent = static_cast<float>(kDefaultCurrentLimit) / kMaxProtocolCurrent;

    static bool is_valid_motor_id(uint8_t motor_id);
    static uint32_t feedback_id_from_motor_id(uint8_t motor_id);
    static uint32_t command_id_from_motor_id(uint8_t motor_id);
    static int16_t clamp_raw_current(float raw_current, int16_t limit);
    static float raw_current_to_current(int16_t raw_current);
    static int16_t current_to_raw_current(float current, int16_t limit);

    void initialize(const M3508MotorParam& param);
    bool register_feedback_callback();
    uint8_t command_slot(uint32_t command_id) const;
    float output_to_motor_torque(float output_torque) const;
    float motor_to_output_torque(float motor_torque) const;
    void update_base_state();

    bsp::FdcanBus* bus_ = nullptr;
    uint8_t motor_id_ = 1U;
    uint32_t tx_id_ = kLowCommandId;
    uint32_t rx_id_ = kFeedbackBaseId + 1U;
    int16_t current_limit_ = kDefaultCurrentLimit;
    int16_t command_current_ = 0;
    float reduction_ratio_ = kDefaultReductionRatio;

    M3508MotorState state_ = {};
    bool feedback_callback_registered_ = false;
};

using RM3508Motor = M3508Motor;
