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

//五连杆并联结构
class LegCalc final : public LegCalcBase {
public:
    explicit LegCalc(double hip_half_distance = 0.11,
                     double upper_link_length = 0.1844,
                     double lower_link_length = 0.3130);

    bool forward_kinematics(const Eigen::Vector2d& joint_position,
                            Eigen::Vector2d& leg_position) const override;
    bool inverse_kinematics(const Eigen::Vector2d& leg_position,
                            Eigen::Vector2d& joint_position) const override;

protected:
    Eigen::Matrix2d calc_jacobian(const Eigen::Vector2d& joint_position) const override;

private:
    template <typename Scalar>
    bool forward_kinematics_impl(const Eigen::Matrix<Scalar, 2, 1>& joint_position,
                                 Eigen::Matrix<Scalar, 2, 1>& leg_position) const;

    double l0_;
    double l1_;
    double l2_;
};

//偏置并联结构
class LegCalc2 : public LegCalcBase {
public:
    LegCalc2(double lab, double lbc, double lcd, double lda, double lce, double lef);

    bool forward_kinematics(const Eigen::Vector2d& joint_position,
                            Eigen::Vector2d& leg_position) const override;
    bool inverse_kinematics(const Eigen::Vector2d& leg_position,
                            Eigen::Vector2d& joint_position) const override;

protected:
    Eigen::Matrix2d calc_jacobian(const Eigen::Vector2d& joint_position) const override;

private:
    template <typename Scalar>
    bool forward_kinematics_impl(const Eigen::Matrix<Scalar, 2, 1>& joint_position,
                                 Eigen::Matrix<Scalar, 2, 1>& leg_position) const;

    template <typename Scalar>
    bool calc_circle_cross_point(
        const Eigen::Vector<Scalar, 2>& circle1,
        const double& r1,
        const Eigen::Vector<Scalar, 2>& circle2,
        const double& r2,
        Eigen::Vector<Scalar, 2>& solution1,
        Eigen::Vector<Scalar, 2>& solution2) const;

    double lab_;
    double lbc_;
    double lcd_;
    double lda_;
    double lce_;
    double lef_;
};

//真串联结构
class LegCalc3 final : public LegCalcBase {
public:
    explicit LegCalc3(const double &l0,const double &l1,double const &l2);

    bool forward_kinematics(const Eigen::Vector2d& joint_position,
                            Eigen::Vector2d& leg_position) const override;
    bool inverse_kinematics(const Eigen::Vector2d& leg_position,
                            Eigen::Vector2d& joint_position) const override;

protected:
    Eigen::Matrix2d calc_jacobian(const Eigen::Vector2d& joint_position) const override;

private:
    template <typename Scalar>
    bool forward_kinematics_impl(const Eigen::Matrix<Scalar, 2, 1>& joint_position,
                                 Eigen::Matrix<Scalar, 2, 1>& leg_position) const;

    double l0_;
    double l1_;
    double l2_;
};