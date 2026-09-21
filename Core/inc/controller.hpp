#pragma once

#include "imubase.hpp"
#include "lqr_gain_debuger.hpp"
#include "motorbase.hpp"
#include "leg_calc.hpp"

#include <Eigen/Dense>

#include <array>
#include <cstdint>
#include <string>

class Controller {
public:
    enum class State : uint8_t {
        VmcTest = 0,
        Recovery = 1,
        Balance = 2,
        Airborne = 3,
    };

    Controller(IMUBase* imu, Motor* lf, Motor* rf, Motor* lb, Motor* rb, Motor* lw, Motor* rw);

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

    bool update(float dt);
    void input(float velocity, float omega, float height = 0.21f, int mode = 0);

    bool set_params(const Params& params);
    bool update_lqr_gain(const LqrGainDebuger::StateWeight& q_diag,
                         const LqrGainDebuger::InputWeight& r_diag,
                         std::string& error);

    State state() const { return state_; }
    Eigen::Vector2d lqr_control() const;

    IMUBase* imu;
    Motor* lf;
    Motor* rf;
    Motor* lb;
    Motor* rb;
    Motor* lw;
    Motor* rw;

private:
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
        bool valid = false;
    };

    static double normalize_angle(double angle);
    static bool orientation_yaw_pitch_roll(const Eigen::Quaternionf& orientation,
                                           double& yaw,
                                           double& pitch,
                                           double& roll);
    static double low_pass_filter(double input,
                                  double alpha,
                                  double& filtered_value,
                                  bool& initialized);
    static double clamp_torque(double torque, double limit);

    bool read_leg_state(Motor* front_hip,
                        Motor* rear_hip,
                        LegState& leg_state) const;
    void set_safe_commands();
    void update_state_machine(double pitch,
                              double left_normal_force,
                              double right_normal_force,
                              double dt);

    LegCalc leg_;
    LqrGainDebuger gain_debuger_;
    LqrGainDebuger::GainMatrix ground_k_mat_ =
        LqrGainDebuger::GainMatrix::Zero();
    LqrGainDebuger::GainMatrix air_k_mat_ =
        LqrGainDebuger::GainMatrix::Zero();
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

    Eigen::Vector2d lqr_control_ = Eigen::Vector2d::Zero();
};
