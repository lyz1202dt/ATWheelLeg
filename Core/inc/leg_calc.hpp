#pragma once

#include <Eigen/Dense>

class LegCalcBase {
public:
    virtual ~LegCalcBase() = default;

    // Input: front/rear hip angles. Output: leg length and leg angle.
    // The leg angle is zero when the leg points vertically down.
    virtual bool forward_kinematics(const Eigen::Vector2d& joint_position,
                                    Eigen::Vector2d& leg_position) const = 0;
    virtual bool inverse_kinematics(const Eigen::Vector2d& leg_position,
                                    Eigen::Vector2d& joint_position) const = 0;

    bool forward_velocity(const Eigen::Vector2d& joint_position,
                          const Eigen::Vector2d& joint_velocity,
                          Eigen::Vector2d& leg_velocity) const;
    bool inverse_velocity(const Eigen::Vector2d& joint_position,
                          const Eigen::Vector2d& leg_velocity,
                          Eigen::Vector2d& joint_velocity) const;
    virtual bool forward_dynamics(const Eigen::Vector2d& joint_position,
                          const Eigen::Vector2d& joint_torque,
                          Eigen::Vector2d& leg_effort) const;
    virtual bool inverse_dynamics(const Eigen::Vector2d& joint_position,
                          const Eigen::Vector2d& leg_effort,
                          Eigen::Vector2d& joint_torque) const;

protected:
    virtual Eigen::Matrix2d calc_jacobian(const Eigen::Vector2d& joint_position) const = 0;
};
