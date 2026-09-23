#pragma once

// 10状态，WBC全身LQR建模控制方案

#include "controllerbase.hpp"
#include "leg_calc.hpp"
#include <Eigen/Dense>
#include <optional>
#include <tuple>


class LegCalc2 : public LegCalcBase {
public:
    LegCalc2(double lab, double lbc, double lcd, double lda, double lce, double lef);

    bool forward_kinematics(const Eigen::Vector2d& joint_position, Eigen::Vector2d& leg_position) const override;
    bool inverse_kinematics(const Eigen::Vector2d& leg_position, Eigen::Vector2d& joint_position) const override;

protected:
    Eigen::Matrix2d calc_jacobian(const Eigen::Vector2d& joint_position) const override;

private:
    template <typename Scalar>
    bool forward_kinematics_impl(const Eigen::Matrix<Scalar, 2, 1>& joint_position, Eigen::Matrix<Scalar, 2, 1>& leg_position) const;

    template <typename Scalar>
    bool calc_circle_cross_point(
        const Eigen::Vector<Scalar, 2>& circle1, const double& r1, const Eigen::Vector<Scalar, 2>& circle2, const double& r2,Eigen::Vector<Scalar, 2> &solution1,Eigen::Vector<Scalar, 2>&solution2) const;

    double lab_;
    double lbc_;
    double lcd_;
    double lda_;
    double lce_;
    double lef_;
};


class Controller10 : public ControllerBase {
public:
    Controller10(IMUBase* imu, Motor* lf, Motor* rf, Motor* lb, Motor* rb, Motor* lw, Motor* rw);
    bool update(float dt) override;
    void input(float velocity, float omega, float height, int mode) override;
};
