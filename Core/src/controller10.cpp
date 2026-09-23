#include "controller10.hpp"
#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>
#include <cmath>


LegCalc2::LegCalc2(double lab, double lbc, double lcd, double lda, double lce, double lef)
    : lab_(lab)
    , lbc_(lbc)
    , lcd_(lcd)
    , lda_(lda)
    , lce_(lce)
    , lef_(lef) {}

bool LegCalc2::forward_kinematics(const Eigen::Vector2d& joint_position, Eigen::Vector2d& leg_position) const {
    return forward_kinematics_impl(joint_position, leg_position);
}

bool LegCalc2::inverse_kinematics(const Eigen::Vector2d& leg_position, Eigen::Vector2d& joint_position) const {
    Eigen::Vector2d Pf(leg_position[0] * std::cos(leg_position[1]), leg_position[0] * std::sin(leg_position[1]));
    Eigen::Vector2d Pc, Pe, Pd, temp;
    bool ret = calc_circle_cross_point(Eigen::Vector2d(0.0, 0.0), lbc_ + lce_, Pf, lef_, Pe, temp);
    if (!ret)
        return false;

    // Pe和temp解的选择
    double beita = std::atan2(Pf.y() - Pe.y(), Pf.x() - Pe.x());
    if (beita > 0.0) {
        Pe    = temp;
        beita = std::atan2(Pf.y() - Pe.y(), Pf.x() - Pe.x());
    }
    joint_position[1] = std::atan2(Pe.y(), Pe.x());

    Pc << lbc_ * std::cos(joint_position[1]), lbc_ * std::sin(joint_position[1]);
    Pd = Pc + Eigen::Vector2d(lcd_ * std::cos(beita), lcd_ * std::sin(beita));

    double lbd     = Pd.norm();
    double cos_fai = (lab_ * lab_ + lbd * lbd - lda_ * lda_) / (2.0 * lab_ * lbd);
    if (cos_fai >= 1.0f)
        return false;
    double fai        = -std::acos(cos_fai);
    double phi        = std::atan2(Pd.y(), Pd.x());
    joint_position[0] = fai + phi;
    return true;
}

Eigen::Matrix2d LegCalc2::calc_jacobian(const Eigen::Vector2d& joint_position) const {
    autodiff::Vector2real q;
    q << joint_position[0], joint_position[1];

    const auto leg_position_function = [this](const autodiff::Vector2real& q_auto) {
        autodiff::Vector2real output;
        forward_kinematics_impl(q_auto, output);
        return output;
    };

    autodiff::Vector2real leg_position;
    Eigen::Matrix2d jacobian;
    autodiff::jacobian(leg_position_function, autodiff::wrt(q), autodiff::at(q), leg_position, jacobian);
    return jacobian;
}

template <typename Scalar>
bool LegCalc2::forward_kinematics_impl(const Eigen::Matrix<Scalar, 2, 1>& joint_position, Eigen::Matrix<Scalar, 2, 1>& leg_position) const {
    Eigen::Vector<Scalar, 2> Pa, Pc, Pd, Pe, Pf, temp;

    Pa << lab_ * cos(joint_position[0]), lab_ * sin(joint_position[0]);
    Pc << lbc_ * cos(joint_position[1]), lbc_ * sin(joint_position[1]);

    const bool ret = calc_circle_cross_point(Pa, lab_, Pc, lbc_, Pd, temp);
    if (!ret)
        return false;

    if (Pd.dot(Pd) < temp.dot(temp)) // 离原点远的点为真实D点
        Pd = temp;

    Scalar lbd       = Pd.norm();
    Scalar cos_alpha = (lbc_ * lbc_ + lcd_ * lcd_ - lbd * lbd) / (2.0 * lbc_ * lcd_);
    if (cos_alpha >= Scalar(1.0))
        return false;
    Scalar alpha = acos(cos_alpha);

    Scalar beita = 3.141592654 - joint_position[1] - alpha;

    Pe << (lbc_ + lce_) * cos(joint_position[1]), (lbc_ + lce_) * sin(joint_position[1]);
    temp << lef_ * cos(beita), lef_ * sin(beita);
    Pf = Pe + temp;

    // 转为极坐标
    leg_position[0] = Pf.norm();
    leg_position[1] = atan2(Pf.y(), Pf.x());

    return true;
}

template <typename Scalar>
bool LegCalc2::calc_circle_cross_point(
    const Eigen::Vector<Scalar, 2>& circle1, const double& r1, const Eigen::Vector<Scalar, 2>& circle2, const double& r2,
    Eigen::Vector<Scalar, 2>& solution1, Eigen::Vector<Scalar, 2>& solution2) const {
    using Vector2 = Eigen::Vector<Scalar, 2>;

    const Scalar x1 = circle1.x();
    const Scalar y1 = circle1.y();

    const Scalar x2 = circle2.x();
    const Scalar y2 = circle2.y();

    const Scalar R1 = Scalar(r1);
    const Scalar R2 = Scalar(r2);

    const Scalar A = Scalar(2) * (x2 - x1);
    const Scalar B = Scalar(2) * (y2 - y1);

    const Scalar C = x1 * x1 + y1 * y1 - x2 * x2 - y2 * y2 + R2 * R2 - R1 * R1;

    if (B != Scalar(0)) {

        const Scalar k = -A / B;
        const Scalar b = -C / B;

        const Scalar d = b - y1;

        const Scalar qa = Scalar(1) + k * k;

        const Scalar qb = Scalar(2) * (k * d - x1);

        const Scalar qc = x1 * x1 + d * d - R1 * R1;

        // 判别式
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
        // 两个圆心重合
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

Controller10::Controller10(IMUBase* imu, Motor* lf, Motor* rf, Motor* lb, Motor* rb, Motor* lw, Motor* rw)
    : ControllerBase(imu, lf, rf, lb, rb, lw, rw) {}


bool Controller10::update(float dt) {}

void Controller10::input(float velocity, float omega, float height, int mode) {}
