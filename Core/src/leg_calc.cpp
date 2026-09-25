#include "leg_calc.hpp"

#include <Eigen/LU>
#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>

namespace {

double clamp_unit(const double value)
{
    return std::clamp(value, -1.0, 1.0);
}

bool reachable(const double distance, const double link_a, const double link_b)
{
    constexpr double kEpsilon = 1.0e-9;
    return distance > kEpsilon && distance <= link_a + link_b &&
           distance >= std::abs(link_a - link_b);
}

bool is_finite_vector(const Eigen::Vector2d& vector)
{
    return vector.allFinite();
}

bool is_finite_matrix(const Eigen::Matrix2d& matrix)
{
    return matrix.array().isFinite().all();
}

bool solve_map(const Eigen::Matrix2d& matrix,
               const Eigen::Vector2d& input,
               Eigen::Vector2d& output)
{
    if (!is_finite_matrix(matrix) || !is_finite_vector(input)) {
        output.setZero();
        return false;
    }

    Eigen::FullPivLU<Eigen::Matrix2d> solver(matrix);
    solver.setThreshold(1.0e-9);
    if (!solver.isInvertible()) {
        output.setZero();
        return false;
    }

    output = solver.solve(input);
    return is_finite_vector(output);
}

} // namespace

bool LegCalcBase::forward_velocity(const Eigen::Vector2d& joint_position,
                                   const Eigen::Vector2d& joint_velocity,
                                   Eigen::Vector2d& leg_velocity) const
{
    if (!is_finite_vector(joint_position) || !is_finite_vector(joint_velocity)) {
        leg_velocity.setZero();
        return false;
    }

    const Eigen::Matrix2d jacobian = calc_jacobian(joint_position);
    if (!is_finite_matrix(jacobian)) {
        leg_velocity.setZero();
        return false;
    }

    leg_velocity = jacobian * joint_velocity;
    return is_finite_vector(leg_velocity);
}

bool LegCalcBase::inverse_velocity(const Eigen::Vector2d& joint_position,
                                   const Eigen::Vector2d& leg_velocity,
                                   Eigen::Vector2d& joint_velocity) const
{
    if (!is_finite_vector(joint_position) || !is_finite_vector(leg_velocity)) {
        joint_velocity.setZero();
        return false;
    }

    return solve_map(calc_jacobian(joint_position), leg_velocity, joint_velocity);
}

bool LegCalcBase::forward_dynamics(const Eigen::Vector2d& joint_position,
                                   const Eigen::Vector2d& joint_torque,
                                   Eigen::Vector2d& leg_effort) const
{
    if (!is_finite_vector(joint_position) || !is_finite_vector(joint_torque)) {
        leg_effort.setZero();
        return false;
    }

    return solve_map(calc_jacobian(joint_position).transpose(), joint_torque, leg_effort);
}

bool LegCalcBase::inverse_dynamics(const Eigen::Vector2d& joint_position,
                                   const Eigen::Vector2d& leg_effort,
                                   Eigen::Vector2d& joint_torque) const
{
    if (!is_finite_vector(joint_position) || !is_finite_vector(leg_effort)) {
        joint_torque.setZero();
        return false;
    }

    const Eigen::Matrix2d jacobian = calc_jacobian(joint_position);
    if (!is_finite_matrix(jacobian)) {
        joint_torque.setZero();
        return false;
    }

    joint_torque = jacobian.transpose() * leg_effort;
    return is_finite_vector(joint_torque);
}

LegCalc::LegCalc(double hip_half_distance,
                 double upper_link_length,
                 double lower_link_length)
    : l0_(hip_half_distance)
    , l1_(upper_link_length)
    , l2_(lower_link_length)
{
}

template <typename Scalar>
bool LegCalc::forward_kinematics_impl(
    const Eigen::Matrix<Scalar, 2, 1>& joint_position,
    Eigen::Matrix<Scalar, 2, 1>& leg_position) const
{
    using Vector2 = Eigen::Matrix<Scalar, 2, 1>;
    using std::atan2;
    using std::cos;
    using std::sin;
    using std::sqrt;

    leg_position.setZero();
    if constexpr (std::is_floating_point_v<Scalar>) {
        if (!std::isfinite(joint_position[0]) || !std::isfinite(joint_position[1])) {
            return false;
        }
    }

    const Scalar l0_scalar = static_cast<Scalar>(l0_);
    const Scalar l1_scalar = static_cast<Scalar>(l1_);
    const Scalar l2_scalar = static_cast<Scalar>(l2_);
    const Scalar epsilon = static_cast<Scalar>(1.0e-9);

    const Vector2 front_elbow(
        l0_scalar + l1_scalar * cos(joint_position[0]),
        l1_scalar * sin(joint_position[0]));
    const Vector2 rear_elbow(
        -l0_scalar + l1_scalar * cos(joint_position[1]),
        l1_scalar * sin(joint_position[1]));

    const Vector2 delta = rear_elbow - front_elbow;
    const Scalar distance_squared = delta.dot(delta);
    if (!(distance_squared > epsilon * epsilon)) {
        return false;
    }

    const Scalar distance = sqrt(distance_squared);
    if (!(distance <= static_cast<Scalar>(2.0) * l2_scalar + epsilon)) {
        return false;
    }

    const Vector2 midpoint = static_cast<Scalar>(0.5) * (front_elbow + rear_elbow);
    const Vector2 half_delta = static_cast<Scalar>(0.5) * delta;
    const Scalar height_squared = l2_scalar * l2_scalar - half_delta.dot(half_delta);
    if (height_squared < -epsilon) {
        return false;
    }

    const Scalar height =
        sqrt(height_squared > static_cast<Scalar>(0.0) ? height_squared : static_cast<Scalar>(0.0));
    const Vector2 normal = Vector2(-delta[1], delta[0]) / distance;
    const Vector2 candidate_a = midpoint + height * normal;
    const Vector2 candidate_b = midpoint - height * normal;
    const Vector2 wheel = candidate_b[1] > candidate_a[1] ? candidate_b : candidate_a;

    leg_position[0] = sqrt(wheel.dot(wheel));
    leg_position[1] = atan2(-wheel[0], wheel[1]);
    return true;
}

bool LegCalc::forward_kinematics(const Eigen::Vector2d& joint_position,
                                 Eigen::Vector2d& leg_position) const
{
    return forward_kinematics_impl(joint_position, leg_position);
}

Eigen::Matrix2d LegCalc::calc_jacobian(const Eigen::Vector2d& joint_position) const
{
    autodiff::Vector2real q;
    q << joint_position[0], joint_position[1];

    const auto leg_position_function = [this](const autodiff::Vector2real& q_auto) {
        autodiff::Vector2real output;
        forward_kinematics_impl(q_auto, output);
        return output;
    };

    autodiff::Vector2real leg_position;
    Eigen::Matrix2d jacobian;
    autodiff::jacobian(
        leg_position_function,
        autodiff::wrt(q),
        autodiff::at(q),
        leg_position,
        jacobian);
    return jacobian;
}

bool LegCalc::inverse_kinematics(const Eigen::Vector2d& leg_position,
                                 Eigen::Vector2d& joint_position) const
{
    const double leg_length = leg_position[0];
    const double leg_angle = leg_position[1];
    const double x = -leg_length * std::sin(leg_angle);
    const double y = leg_length * std::cos(leg_angle);

    const double front_dx = x - l0_;
    const double front_dy = y;
    const double front_distance = std::hypot(front_dx, front_dy);
    if (!reachable(front_distance, l1_, l2_)) {
        return false;
    }
    const double front_angle = std::atan2(front_dy, front_dx);
    const double front_offset = std::acos(clamp_unit(
        (l1_ * l1_ + front_distance * front_distance - l2_ * l2_) /
        (2.0 * l1_ * front_distance)));

    const double rear_dx = x + l0_;
    const double rear_dy = y;
    const double rear_distance = std::hypot(rear_dx, rear_dy);
    if (!reachable(rear_distance, l1_, l2_)) {
        return false;
    }
    const double rear_angle = std::atan2(rear_dy, rear_dx);
    const double rear_offset = std::acos(clamp_unit(
        (l1_ * l1_ + rear_distance * rear_distance - l2_ * l2_) /
        (2.0 * l1_ * rear_distance)));

    joint_position = {front_angle - front_offset, rear_angle + rear_offset};
    return joint_position.allFinite();
}

LegCalc2::LegCalc2(double lab, double lbc, double lcd, double lda, double lce, double lef)
    : lab_(lab)
    , lbc_(lbc)
    , lcd_(lcd)
    , lda_(lda)
    , lce_(lce)
    , lef_(lef)
{
}

bool LegCalc2::forward_kinematics(const Eigen::Vector2d& joint_position,
                                  Eigen::Vector2d& leg_position) const
{
    return forward_kinematics_impl(joint_position, leg_position);
}

bool LegCalc2::inverse_kinematics(const Eigen::Vector2d& leg_position,
                                  Eigen::Vector2d& joint_position) const
{
    Eigen::Vector2d Pf(
        leg_position[0] * std::cos(leg_position[1]),
        leg_position[0] * std::sin(leg_position[1]));
    Eigen::Vector2d Pc;
    Eigen::Vector2d Pe;
    Eigen::Vector2d Pd;
    Eigen::Vector2d temp;
    const bool ret = calc_circle_cross_point(
        Eigen::Vector2d(0.0, 0.0), lbc_ + lce_, Pf, lef_, Pe, temp);
    if (!ret) {
        return false;
    }

    // Select the intersection that matches the configured mechanism branch.
    double beita = std::atan2(Pf.y() - Pe.y(), Pf.x() - Pe.x());
    if (beita > 0.0) {
        Pe = temp;
        beita = std::atan2(Pf.y() - Pe.y(), Pf.x() - Pe.x());
    }
    joint_position[1] = std::atan2(Pe.y(), Pe.x());

    Pc << lbc_ / (lbc_ + lce_) * Pe;
    Pd = Pc + lcd_ / lef_ * (Pf - Pe);

    const double lbd = Pd.norm();
    const double cos_fai = (lab_ * lab_ + lbd * lbd - lda_ * lda_) /
                           (2.0 * lab_ * lbd);
    if (std::abs(cos_fai) >= 1.0) {
        return false;
    }
    const double fai = -std::acos(cos_fai);
    const double phi = std::atan2(Pd.y(), Pd.x());
    joint_position[0] = fai + phi;
    return true;
}

Eigen::Matrix2d LegCalc2::calc_jacobian(const Eigen::Vector2d& joint_position) const
{
    autodiff::Vector2real q;
    q << joint_position[0], joint_position[1];

    const auto leg_position_function = [this](const autodiff::Vector2real& q_auto) {
        autodiff::Vector2real output;
        forward_kinematics_impl(q_auto, output);
        return output;
    };

    autodiff::Vector2real leg_position;
    Eigen::Matrix2d jacobian;
    autodiff::jacobian(
        leg_position_function,
        autodiff::wrt(q),
        autodiff::at(q),
        leg_position,
        jacobian);
    return jacobian;
}

template <typename Scalar>
bool LegCalc2::forward_kinematics_impl(
    const Eigen::Matrix<Scalar, 2, 1>& joint_position,
    Eigen::Matrix<Scalar, 2, 1>& leg_position) const
{
    Eigen::Vector<Scalar, 2> Pa;
    Eigen::Vector<Scalar, 2> Pc;
    Eigen::Vector<Scalar, 2> Pd;
    Eigen::Vector<Scalar, 2> Pe;
    Eigen::Vector<Scalar, 2> Pf;
    Eigen::Vector<Scalar, 2> temp;

    Pa << lab_ * cos(joint_position[0]), lab_ * sin(joint_position[0]);
    Pc << lbc_ * cos(joint_position[1]), lbc_ * sin(joint_position[1]);

    const bool ret = calc_circle_cross_point(Pa, lda_, Pc, lcd_, Pd, temp);
    if (!ret) {
        return false;
    }

    if (Pd.dot(Pd) < temp.dot(temp)) {
        Pd = temp;
    }

    const Scalar lbd = Pd.norm();
    const Scalar cos_alpha =
        (lbc_ * lbc_ + lcd_ * lcd_ - lbd * lbd) / (2.0 * lbc_ * lcd_);
    if (abs(cos_alpha) >= Scalar(1.0)) {
        return false;
    }
    const Scalar alpha = acos(cos_alpha);
    const Scalar beita = joint_position[1] + alpha - 3.141592654;

    Pe << (lbc_ + lce_) * cos(joint_position[1]),
        (lbc_ + lce_) * sin(joint_position[1]);
    temp << lef_ * cos(beita), lef_ * sin(beita);
    Pf = Pe + temp;

    leg_position[0] = Pf.norm();
    leg_position[1] = atan2(Pf.y(), Pf.x());
    return true;
}

template <typename Scalar>
bool LegCalc2::calc_circle_cross_point(
    const Eigen::Vector<Scalar, 2>& circle1,
    const double& r1,
    const Eigen::Vector<Scalar, 2>& circle2,
    const double& r2,
    Eigen::Vector<Scalar, 2>& solution1,
    Eigen::Vector<Scalar, 2>& solution2) const
{
    const Scalar x1 = circle1.x();
    const Scalar y1 = circle1.y();
    const Scalar x2 = circle2.x();
    const Scalar y2 = circle2.y();
    const Scalar R1 = Scalar(r1);
    const Scalar R2 = Scalar(r2);

    const Scalar A = Scalar(2) * (x2 - x1);
    const Scalar B = Scalar(2) * (y2 - y1);
    const Scalar C =
        x1 * x1 + y1 * y1 - x2 * x2 - y2 * y2 + R2 * R2 - R1 * R1;

    if (B != Scalar(0)) {
        const Scalar k = -A / B;
        const Scalar b = -C / B;
        const Scalar d = b - y1;
        const Scalar qa = Scalar(1) + k * k;
        const Scalar qb = Scalar(2) * (k * d - x1);
        const Scalar qc = x1 * x1 + d * d - R1 * R1;
        const Scalar delta = qb * qb - Scalar(4) * qa * qc;
        if (delta < Scalar(0)) {
            return false;
        }

        const Scalar sqrt_delta = sqrt(delta);
        const Scalar x_cross1 = (-qb + sqrt_delta) / (Scalar(2) * qa);
        const Scalar x_cross2 = (-qb - sqrt_delta) / (Scalar(2) * qa);
        const Scalar y_cross1 = k * x_cross1 + b;
        const Scalar y_cross2 = k * x_cross2 + b;
        solution1 << x_cross1, y_cross1;
        solution2 << x_cross2, y_cross2;
        return true;
    }

    if (A == Scalar(0)) {
        return false;
    }

    const Scalar x = -C / A;
    const Scalar dy = x - x1;
    const Scalar y_offset = sqrt(R1 * R1 - dy * dy);
    solution1.x() = x;
    solution1.y() = y1 + y_offset;
    solution2.x() = x;
    solution2.y() = y1 - y_offset;
    return true;
}

LegCalc3::LegCalc3(const double& l0, const double& l1, const double& l2)
    : l0_(l0)
    , l1_(l1)
    , l2_(l2)
{
}

template <typename Scalar>
bool LegCalc3::forward_kinematics_impl(
    const Eigen::Matrix<Scalar, 2, 1>& joint_position,
    Eigen::Matrix<Scalar, 2, 1>& leg_position) const
{
    using std::atan2;
    using std::cos;
    using std::sin;
    using std::sqrt;

    leg_position.setZero();
    if (!std::isfinite(l0_) || !std::isfinite(l1_) || !std::isfinite(l2_) ||
        l0_ < 0.0 || l1_ <= 0.0 || l2_ <= 0.0) {
        return false;
    }

    const Scalar first_link_angle = joint_position[0];
    const Scalar second_link_angle = first_link_angle + joint_position[1];
    const Scalar l0_scalar = static_cast<Scalar>(l0_);
    const Scalar l1_scalar = static_cast<Scalar>(l1_);
    const Scalar l2_scalar = static_cast<Scalar>(l2_);

    // x points down and y points right. The knee angle is relative to the
    // first link, so the second link orientation is q0 + q1.
    const Scalar x = l0_scalar
                   + l1_scalar * cos(first_link_angle)
                   + l2_scalar * cos(second_link_angle);
    const Scalar y = l1_scalar * sin(first_link_angle)
                   + l2_scalar * sin(second_link_angle);
    const Scalar length_squared = x * x + y * y;
    const Scalar length = sqrt(length_squared);

    if (!(length > static_cast<Scalar>(1.0e-12))) {
        return false;
    }

    leg_position[0] = length;
    leg_position[1] = atan2(y, x);
    return true;
}

bool LegCalc3::forward_kinematics(const Eigen::Vector2d& joint_position,
                                  Eigen::Vector2d& leg_position) const
{
    if (!is_finite_vector(joint_position)) {
        leg_position.setZero();
        return false;
    }

    return forward_kinematics_impl(joint_position, leg_position);
}

bool LegCalc3::inverse_kinematics(const Eigen::Vector2d& leg_position,
                                  Eigen::Vector2d& joint_position) const
{
    joint_position.setZero();
    if (!is_finite_vector(leg_position) || !std::isfinite(l0_) ||
        !std::isfinite(l1_) || !std::isfinite(l2_) || l0_ < 0.0 ||
        l1_ <= 0.0 || l2_ <= 0.0) {
        return false;
    }

    constexpr double kEpsilon = 1.0e-9;
    const double leg_length = leg_position[0];
    const double leg_angle = leg_position[1];
    if (leg_length <= kEpsilon) {
        return false;
    }

    const double target_x = leg_length * std::cos(leg_angle);
    const double target_y = leg_length * std::sin(leg_angle);
    const double x = target_x - l0_;
    const double y = target_y;
    const double distance_squared = x * x + y * y;
    const double distance = std::sqrt(distance_squared);
    const double min_distance = std::abs(l1_ - l2_);
    const double max_distance = l1_ + l2_;

    if (!std::isfinite(distance) ||
        distance < min_distance - kEpsilon ||
        distance > max_distance + kEpsilon ||
        distance <= kEpsilon) {
        return false;
    }

    double cos_knee = (distance_squared - l1_ * l1_ - l2_ * l2_) /
                      (2.0 * l1_ * l2_);
    if (cos_knee < -1.0 - kEpsilon || cos_knee > 1.0 + kEpsilon) {
        return false;
    }
    cos_knee = std::clamp(cos_knee, -1.0, 1.0);

    // Select the branch with a non-negative relative knee angle.
    const double knee_angle = std::acos(cos_knee);
    const double hip_angle = std::atan2(y, x) -
                             std::atan2(l2_ * std::sin(knee_angle),
                                        l1_ + l2_ * std::cos(knee_angle));

    joint_position << hip_angle, knee_angle;
    return is_finite_vector(joint_position);
}

Eigen::Matrix2d LegCalc3::calc_jacobian(const Eigen::Vector2d& joint_position) const
{
    Eigen::Matrix2d jacobian;
    jacobian.setConstant(std::numeric_limits<double>::quiet_NaN());

    if (!is_finite_vector(joint_position) || !std::isfinite(l0_) ||
        !std::isfinite(l1_) || !std::isfinite(l2_) || l0_ < 0.0 ||
        l1_ <= 0.0 || l2_ <= 0.0) {
        return jacobian;
    }

    const double first_link_angle = joint_position[0];
    const double second_link_angle = first_link_angle + joint_position[1];
    const double x = l0_ + l1_ * std::cos(first_link_angle) +
                     l2_ * std::cos(second_link_angle);
    const double y = l1_ * std::sin(first_link_angle) +
                     l2_ * std::sin(second_link_angle);
    const double length_squared = x * x + y * y;
    if (!std::isfinite(length_squared) || length_squared <= 1.0e-24) {
        return jacobian;
    }

    const double dx_dq0 = -l1_ * std::sin(first_link_angle) -
                          l2_ * std::sin(second_link_angle);
    const double dy_dq0 = l1_ * std::cos(first_link_angle) +
                          l2_ * std::cos(second_link_angle);
    const double dx_dq1 = -l2_ * std::sin(second_link_angle);
    const double dy_dq1 = l2_ * std::cos(second_link_angle);

    const double length = std::sqrt(length_squared);
    jacobian(0, 0) = (x * dx_dq0 + y * dy_dq0) / length;
    jacobian(0, 1) = (x * dx_dq1 + y * dy_dq1) / length;
    jacobian(1, 0) = (x * dy_dq0 - y * dx_dq0) / length_squared;
    jacobian(1, 1) = (x * dy_dq1 - y * dx_dq1) / length_squared;
    return jacobian;
}
