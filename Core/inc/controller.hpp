#pragma once

#include "imubase.hpp"
#include "tools/leg_calc.hpp"
#include "motorbase.hpp"

#include <Eigen/Dense>

#include <atomic>
#include <cstdint>

class Controller {
public:
    enum class State : uint8_t {
        VmcTest = 0,
        Recovery = 1,
        Balance = 2,
        Airborne = 3,
    };

    Controller(IMUBase* imu, Motor* lf, Motor* rf, Motor* lb, Motor* rb, Motor* lw, Motor* rw);

    bool update(float dt);
    void input(float velocity, float omega, float height = 0.21f, int mode = 0);

    bool set_K(const Eigen::Matrix<double, 2, 6>& gain);
    bool calculate_lqr_gain(const volatile float q_diag[6],
                            const volatile float r_diag[2],
                            Eigen::Matrix<double, 2, 6>& gain) const;

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

    struct LegState {
        Eigen::Vector2d joint_position = Eigen::Vector2d::Zero();
        Eigen::Vector2d joint_velocity = Eigen::Vector2d::Zero();
        Eigen::Vector2d leg_position = Eigen::Vector2d::Zero();
        Eigen::Vector2d leg_velocity = Eigen::Vector2d::Zero();
        Eigen::Vector2d leg_effort = Eigen::Vector2d::Zero();
        bool valid = false;
    };

    static double normalize_angle(double angle);
    static double normalized_pitch(const Eigen::Quaternionf& orientation);
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
    void read_gain(Eigen::Matrix<double, 2, 6>& gain,
                   Eigen::Matrix<double, 2, 6>& air_gain) const;
    bool solve_lqr_gain(const volatile float q_diag[6],
                        const volatile float r_diag[2],
                        Eigen::Matrix<double, 2, 6>& gain) const;

    LegCalc leg_;
    State state_ = State::Recovery;
    int requested_mode_ = 0;
    double state_switch_elapsed_ = 0.0;

    double expected_velocity_ = 0.0;
    double expected_omega_ = 0.0;
    double expected_height_ = 0.21;
    double expected_position_ = 0.0;

    double vmc_kp_ = 600.0;
    double vmc_kd_ = 40.0;
    double leg_angle_difference_kp_ = 30.0;
    double leg_angle_difference_kd_ = 4.0;
    double wheel_difference_kp_ = 0.1;
    double wheel_difference_ki_ = 0.002;
    double wheel_spin_error_integral_ = 0.0;

    double filtered_dx_ = 0.0;
    double filtered_dtheta_ = 0.0;
    double filtered_dphi_ = 0.0;
    bool dx_filter_initialized_ = false;
    bool dtheta_filter_initialized_ = false;
    bool dphi_filter_initialized_ = false;

    Eigen::Matrix<double, 2, 6> gain_ = Eigen::Matrix<double, 2, 6>::Zero();
    Eigen::Matrix<double, 2, 6> air_gain_ = Eigen::Matrix<double, 2, 6>::Zero();
    Eigen::Vector2d lqr_control_ = Eigen::Vector2d::Zero();
    std::atomic<uint32_t> gain_sequence_{0U};
};

extern "C" {
extern volatile float lqr_q_diag[6];
extern volatile float lqr_r_diag[2];
extern volatile uint32_t lqr_gain_update_request;
}
