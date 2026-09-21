#pragma once


#include <cmath>
#include <Eigen/Dense>
#include <stdexcept>
#include <type_traits>
#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>

class LegCalc{
public:
    LegCalc(const double &l0,const double &l1,const double &l2);

    //正运动学，输入{前髋关节角, 后髋关节角}，输出{腿长, 腿角度}，腿竖直向下为0
    template <typename Scalar>
    bool forward_kinematics(const Eigen::Matrix<Scalar, 2, 1> &rad,Eigen::Matrix<Scalar, 2, 1> &leg) const {
        using Vector2 = Eigen::Matrix<Scalar, 2, 1>;
        using std::atan2;
        using std::cos;
        using std::sin;
        using std::sqrt;

        leg.setZero();
        if constexpr (std::is_floating_point_v<Scalar>) {
            if (!std::isfinite(rad[0]) || !std::isfinite(rad[1])) {
                return false;
            }
        }

        const Scalar l0_scalar = static_cast<Scalar>(l0);
        const Scalar l1_scalar = static_cast<Scalar>(l1);
        const Scalar l2_scalar = static_cast<Scalar>(l2);
        const Scalar epsilon = static_cast<Scalar>(1e-9);

        const Vector2 front_elbow(l0_scalar + l1_scalar * cos(rad[0]), l1_scalar * sin(rad[0]));
        const Vector2 rear_elbow(-l0_scalar + l1_scalar * cos(rad[1]), l1_scalar * sin(rad[1]));

        const Vector2 delta = rear_elbow - front_elbow;
        const Scalar distance_sq = delta.dot(delta);
        if (!(distance_sq > epsilon * epsilon)) {
            return false;
        }

        const Scalar distance = sqrt(distance_sq);
        if (!(distance <= static_cast<Scalar>(2.0) * l2_scalar + epsilon)) {
            return false;
        }

        const Vector2 mid = static_cast<Scalar>(0.5) * (front_elbow + rear_elbow);
        const Vector2 half_delta = static_cast<Scalar>(0.5) * delta;
        const Scalar height_sq = l2_scalar * l2_scalar - half_delta.dot(half_delta);
        if (height_sq < -epsilon) {
            return false;
        }

        const Scalar height = sqrt(height_sq > static_cast<Scalar>(0.0) ? height_sq : static_cast<Scalar>(0.0));
        const Vector2 normal = Vector2(-delta[1], delta[0]) / distance;

        const Vector2 candidate_a = mid + height * normal;
        const Vector2 candidate_b = mid - height * normal;
        Vector2 wheel = candidate_a;
        if (candidate_b[1] > candidate_a[1]) {
            wheel = candidate_b;
        }

        leg[0] = sqrt(wheel.dot(wheel));
        leg[1] = atan2(-wheel[0], wheel[1]);
        return true;
    }

    //逆运动学，输入{腿长, 腿角度}，输出{前髋关节角, 后髋关节角}，腿竖直向下为0
    bool inverse_kinematics(const Eigen::Matrix<double, 2, 1> &leg,Eigen::Matrix<double, 2, 1> &rad) const;

    //正速度映射，输入{前髋关节角, 后髋关节角}和{前髋角速度, 后髋角速度}，输出{腿长速度, 腿角速度}
    bool forward_velocity(const Eigen::Vector2d &rad,const Eigen::Vector2d &joint_velocity,Eigen::Vector2d &leg_velocity) const;

    //逆速度映射，输入{前髋关节角, 后髋关节角}和{腿长速度, 腿角速度}，输出{前髋角速度, 后髋角速度}
    bool inverse_velocity(const Eigen::Vector2d &rad,const Eigen::Vector2d &leg_velocity,Eigen::Vector2d &joint_velocity) const;

    //正动力学，输入{t1,t2}，输出{F,T}
    bool forward_dynamics(const Eigen::Vector2d &rad,const Eigen::Vector2d &torque,Eigen::Vector2d &effort) const;
    
    //逆动力学，输入{F,T}，输出{t1,t2}
    bool inverse_dynamics(const Eigen::Vector2d &rad,const Eigen::Vector2d &effort,Eigen::Vector2d &torque) const;
    
private:
    Eigen::Matrix2d calc_jacobian(const Eigen::Vector2d& radian) const;

    double l0,l1,l2;
};
