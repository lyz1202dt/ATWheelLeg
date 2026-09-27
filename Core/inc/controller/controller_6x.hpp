#pragma once

#include "imubase.hpp"
#include "motorbase.hpp"
#include "controllerbase.hpp"
#include "tools/leg_calc.hpp"

#include <Eigen/Dense>

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <utility>

using LqrGainMatrix = Eigen::Matrix<double, 2, 6>;
using LqrStateVector = Eigen::Matrix<double, 6, 1>;
using LqrStateWeight = std::array<double, 6>;
using LqrInputWeight = std::array<double, 2>;

class Controller : public ControllerBase {
public:
    enum class State : uint8_t {
        VmcTest = 0,
        Recovery = 1,
        Balance = 2,
        Airborne = 3,
    };

    Controller(IMUBase* imu,
               Motor* lf,
               Motor* rf,
               Motor* lb,
               Motor* rb,
               Motor* lw,
               Motor* rw,
               std::function<bool(double, LqrGainMatrix&)> gain_provider,
               DebugLogger debug_logger = nullptr);

    struct Params {
        double body_width = 0.34;
        double base_link_com_height = 0.1265;
        double centrifugal_accel_filter_alpha = 0.2;
        double centrifugal_force_ff_gain = 1.0;
        double centrifugal_force_ff_limit = 40.0;
        double leg_exp_length = 0.28;
        double vmc_kp = 600.0;
        double vmc_kd = 40.0;
        double leg_angle_diff_kp = 30.0;
        double leg_angle_diff_kd = 4.0;
        double wheel_diff_kp = 0.1;
        double wheel_diff_ki = 0.002;
    };

    bool update(uint64_t ms) override;
    void input(float velocity, float omega, float height = 0.21f, int mode = 0) override;

    bool set_params(const Params& params);

    State state() const { return state_; }
    Eigen::Vector2d lqr_control() const;
    

private:
    using StateVector = LqrStateVector;

    static constexpr double kPi                    = 3.14159265358979323846;
    static constexpr double kDefaultDt             = 0.001;
    static constexpr double kStateSwitchDelay      = 0.3;
    static constexpr double kPitchLimit             = 0.3;
    static constexpr double kContactForceThreshold = 8.0;
    static constexpr double kHipTorqueLimit        = 12.0;
    static constexpr double kWheelTorqueLimit      = 2.0;

    static constexpr double kHipHalfDistance = 0.11;
    static constexpr double kUpperLinkLength = 0.1844;
    static constexpr double kLowerLinkLength = 0.3130;
    static constexpr double kWheelRadius = 0.1;
    static constexpr double kWheelSpinIntegralLimit = 20.0;
    static constexpr double kBaselinkMass = 2.30;

    struct LegState {
        Eigen::Vector2d joint_position = Eigen::Vector2d::Zero();
        Eigen::Vector2d joint_velocity = Eigen::Vector2d::Zero();
        Eigen::Vector2d leg_position = Eigen::Vector2d::Zero();
        Eigen::Vector2d leg_velocity = Eigen::Vector2d::Zero();
        Eigen::Vector2d leg_effort = Eigen::Vector2d::Zero();
    };

    static double normalize_angle(double angle);
    static bool orientation_yaw_pitch_roll(const Eigen::Quaternionf& orientation,
                                           Eigen::Quaterniond& normalized_orientation,
                                           double& yaw,
                                           double& pitch,
                                           double& roll);
    static double low_pass_filter(double input,
                                  double alpha,
                                  double& filtered_value,
                                  bool& initialized);
    static double clamp_torque(double torque, double limit);

    bool read_leg_position(Motor* front_hip,
                        Motor* rear_hip,
                        LegState& leg_position) const;
    bool send_recovery_commands();
    bool send_torque_commands(const Eigen::Vector2d& left_joint_torque,
                              const Eigen::Vector2d& right_joint_torque,
                              double left_wheel_torque,
                              double right_wheel_torque);
    void set_safe_commands();
    void update_state_machine(double pitch,
                              double left_normal_force,
                              double right_normal_force,
                              double dt);

    FiveBarLegCalc leg_;
    std::function<bool(double, LqrGainMatrix&)> gain_provider_;
    Params params_;
    State state_ = State::Recovery;
    int requested_mode_ = 0;
    double state_switch_elapsed_ = 0.0;

    double expected_velocity_ = 0.0;
    double expected_omega_ = 0.0;
    double expected_height_ = 0.28;
    double expected_roll_ = 0.0;
    double expected_position_ = 0.0;

    double wheel_spin_error_integral_ = 0.0;

    double filtered_dx_ = 0.0;
    double filtered_dtheta_ = 0.0;
    double filtered_dphi_ = 0.0;
    bool dx_filter_initialized_ = false;
    bool dtheta_filter_initialized_ = false;
    bool dphi_filter_initialized_ = false;
    double filtered_lateral_acceleration_ = 0.0;
    bool lateral_acceleration_filter_initialized_ = false;
    uint64_t last_update_ms_ = 0U;
    bool update_time_initialized_ = false;

    Eigen::Vector2d lqr_control_ = Eigen::Vector2d::Zero();
};
