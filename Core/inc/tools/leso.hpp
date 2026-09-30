#pragma once

#include <Eigen/Dense>


// 状态向量维度，输入向量维度，观测向量维度
template <typename Scalar, int StateDim, int InputDim, int OutputDim, int DisturbanceDim>
class LESO {
public:
    static constexpr int ExtendedStateDim = StateDim + DisturbanceDim;


    using State         = Eigen::Matrix<Scalar, StateDim, 1>;
    using Input         = Eigen::Matrix<Scalar, InputDim, 1>;
    using Output        = Eigen::Matrix<Scalar, OutputDim, 1>;
    using Disturbance   = Eigen::Matrix<Scalar, DisturbanceDim, 1>;
    using ExtendedState = Eigen::Matrix<Scalar, ExtendedStateDim, 1>;
    using Poles         = Eigen::Matrix<Scalar, ExtendedStateDim, 1>;

    using A = Eigen::Matrix<Scalar, StateDim, StateDim>;
    using B = Eigen::Matrix<Scalar, StateDim, InputDim>;
    using E = Eigen::Matrix<Scalar, StateDim, DisturbanceDim>;
    using C = Eigen::Matrix<Scalar, OutputDim, StateDim>;
    using L = Eigen::Matrix<Scalar, ExtendedStateDim, OutputDim>;

    using Ae = Eigen::Matrix<Scalar, ExtendedStateDim, ExtendedStateDim>;
    using Be = Eigen::Matrix<Scalar, ExtendedStateDim, InputDim>;
    using Ce = Eigen::Matrix<Scalar, OutputDim, ExtendedStateDim>;


    LESO(const A& A, const B& B, const E& E, const C& C, const Poles& poles){
        static_assert(OutputDim == 1, "Pole-placement LESO currently supports single-output systems only.");

        Ae_.setZero();  //清空
        Ae_.template block<StateDim,StateDim>(0,0)=A;
        Ae_.template block<StateDim,DisturbanceDim>(0,StateDim)=E;
        // 扰动假设为慢变/常值：d_dot = 0。

        Be_.setZero();
        Be_.template block<StateDim,InputDim>(0,0)=B;

        Ce_.setZero();
        Ce_.template block<OutputDim,StateDim>(0,0)=C;

        observer_gain_valid_ = calculate_observer_gain(poles);
    }

    LESO(const A& A, const B& B, const E& E, const C& C, Scalar bandwidth){
        Ae_.setZero();  //清空
        Ae_.template block<StateDim,StateDim>(0,0)=A;
        Ae_.template block<StateDim,DisturbanceDim>(0,StateDim)=E;
        // 扰动假设为慢变/常值：d_dot = 0。

        Be_.setZero();
        Be_.template block<StateDim,InputDim>(0,0)=B;

        Ce_.setZero();
        Ce_.template block<OutputDim,StateDim>(0,0)=C;

        observer_gain_valid_ = calculate_full_state_observer_gain(E, bandwidth);
    }

    void reset(const ExtendedState& x) {
        xe_ = x;
    }

    void reset(const State& x, const Disturbance& z = Disturbance::Zero()) {
        xe_.template head<StateDim>() = x;
        xe_.template tail<DisturbanceDim>() = z;
    }

    bool update(const Output &y,const Input &u,Scalar dt,Disturbance &z) {
        if (!observer_gain_valid_ || dt <= Scalar(0) || !y.allFinite() || !u.allFinite()) {
            return false;
        }

        const Output ey = y - Ce_ * xe_;
        xe_ += dt * (Ae_ * xe_ + Be_ * u + L_ * ey);

        if (!xe_.allFinite()) {
            return false;
        }

        z = xe_.template tail<DisturbanceDim>();
        return true;
    }

    const ExtendedState& extended_state() const { return xe_; }
    State state() const { return xe_.template head<StateDim>(); }
    Disturbance disturbance() const { return xe_.template tail<DisturbanceDim>(); }
    const L& observer_gain() const { return L_; }
    bool observer_gain_valid() const { return observer_gain_valid_; }

private:
    using MatrixN = Eigen::Matrix<Scalar, ExtendedStateDim, ExtendedStateDim>;
    using VectorN = Eigen::Matrix<Scalar, ExtendedStateDim, 1>;
    using RowN = Eigen::Matrix<Scalar, 1, ExtendedStateDim>;
    using PolynomialCoefficients = Eigen::Matrix<Scalar, ExtendedStateDim + 1, 1>;

    bool calculate_observer_gain(const Poles& poles) {
        if (!poles.allFinite()) {
            L_.setZero();
            return false;
        }

        const MatrixN ad = Ae_.transpose();
        const VectorN bd = Ce_.transpose();

        MatrixN controllability;
        VectorN column = bd;
        for (int index = 0; index < ExtendedStateDim; ++index) {
            controllability.col(index) = column;
            column = ad * column;
        }

        Eigen::FullPivLU<MatrixN> controllability_lu(controllability);
        if (!controllability_lu.isInvertible()) {
            L_.setZero();
            return false;
        }

        PolynomialCoefficients polynomial = PolynomialCoefficients::Zero();
        polynomial(0) = Scalar(1);
        int degree = 0;
        for (int pole_index = 0; pole_index < ExtendedStateDim; ++pole_index) {
            for (int coeff_index = degree + 1; coeff_index > 0; --coeff_index) {
                polynomial(coeff_index) = polynomial(coeff_index - 1) - poles(pole_index) * polynomial(coeff_index);
            }
            polynomial(0) *= -poles(pole_index);
            ++degree;
        }

        MatrixN phi = MatrixN::Zero();
        MatrixN power = MatrixN::Identity();
        for (int degree_index = 0; degree_index < ExtendedStateDim; ++degree_index) {
            phi += polynomial(degree_index) * power;
            power = power * ad;
        }
        phi += power;

        RowN selector = RowN::Zero();
        selector(ExtendedStateDim - 1) = Scalar(1);

        const RowN gain = selector * controllability_lu.solve(phi);
        L_ = gain.transpose();
        if (!L_.allFinite()) {
            L_.setZero();
            return false;
        }
        return true;
    }

    bool calculate_full_state_observer_gain(const E& disturbance_map, Scalar bandwidth) {
        if constexpr (OutputDim != StateDim) {
            (void)disturbance_map;
            (void)bandwidth;
            L_.setZero();
            return false;
        } else {
            if (!(bandwidth > Scalar(0)) || !disturbance_map.allFinite()) {
                L_.setZero();
                return false;
            }

            Eigen::FullPivLU<E> disturbance_lu(disturbance_map);
            if (disturbance_lu.rank() < DisturbanceDim) {
                L_.setZero();
                return false;
            }

            const Eigen::Matrix<Scalar, DisturbanceDim, DisturbanceDim> gram =
                disturbance_map.transpose() * disturbance_map;
            Eigen::LDLT<Eigen::Matrix<Scalar, DisturbanceDim, DisturbanceDim>> gram_ldlt(gram);
            if (gram_ldlt.info() != Eigen::Success) {
                L_.setZero();
                return false;
            }

            const Eigen::Matrix<Scalar, DisturbanceDim, StateDim> disturbance_pinv =
                gram_ldlt.solve(disturbance_map.transpose());
            if (!disturbance_pinv.allFinite()) {
                L_.setZero();
                return false;
            }

            L_.setZero();
            L_.template topRows<StateDim>() =
                Scalar(2) * bandwidth * Eigen::Matrix<Scalar, StateDim, StateDim>::Identity();
            L_.template bottomRows<DisturbanceDim>() = bandwidth * bandwidth * disturbance_pinv;
            return L_.allFinite();
        }
    }

    Ae Ae_;
    Be Be_;
    Ce Ce_;
    L L_{L::Zero()};
    ExtendedState xe_{ExtendedState::Zero()};
    bool observer_gain_valid_{false};
};
