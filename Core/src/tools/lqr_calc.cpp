#include "tools/lqr_calc.hpp"

#include <Eigen/Eigenvalues>

#include <cmath>
#include <vector>

LQRCalc::LQRCalc(const Matrix& a, const Matrix& b)
    : a_(a)
    , b_(b) {}

bool LQRCalc::validate_system_matrices(const Matrix& a,
                                       const Matrix& b,
                                       std::string& error) const {
    error.clear();
    if (a.rows() <= 0 || a.cols() <= 0 || a.rows() != a.cols()) {
        error = "A must be a non-empty square matrix";
        return false;
    }
    if (b.rows() != a.rows() || b.cols() <= 0) {
        error = "B must have the same row count as A and at least one column";
        return false;
    }
    if (!a.allFinite() || !b.allFinite()) {
        error = "A and B must contain only finite values";
        return false;
    }
    return true;
}

bool LQRCalc::set_a(const Matrix& a, std::string& error) {
    if (!validate_system_matrices(a, b_, error)) {
        return false;
    }
    a_ = a;
    return true;
}

bool LQRCalc::set_b(const Matrix& b, std::string& error) {
    if (!validate_system_matrices(a_, b, error)) {
        return false;
    }
    b_ = b;
    return true;
}

bool LQRCalc::set_system_matrices(const Matrix& a,
                                  const Matrix& b,
                                  std::string& error) {
    if (!validate_system_matrices(a, b, error)) {
        return false;
    }
    a_ = a;
    b_ = b;
    return true;
}

bool LQRCalc::calculate(const Vector& q_diag,
                        const Vector& r_diag,
                        Matrix& gain,
                        std::string& error) const {
    error.clear();
    gain.resize(0, 0);

    if (!validate_system_matrices(a_, b_, error)) {
        return false;
    }

    const Eigen::Index state_size = a_.rows();
    const Eigen::Index input_size = b_.cols();
    if (q_diag.size() != state_size) {
        error = "Q must contain one diagonal weight for each state";
        return false;
    }
    if (r_diag.size() != input_size) {
        error = "R must contain one diagonal weight for each input";
        return false;
    }

    Eigen::MatrixXd q = Eigen::MatrixXd::Zero(state_size, state_size);
    Eigen::MatrixXd r_inverse =
        Eigen::MatrixXd::Zero(input_size, input_size);
    for (Eigen::Index index = 0; index < state_size; ++index) {
        const double weight = q_diag[index];
        if (!std::isfinite(weight) || weight < 0.0) {
            error = "Q diagonal values must be finite and greater than or equal to 0";
            return false;
        }
        q(index, index) = weight;
    }
    for (Eigen::Index index = 0; index < input_size; ++index) {
        const double weight = r_diag[index];
        if (!std::isfinite(weight) || weight <= 0.0) {
            error = "R diagonal values must be finite and greater than 0";
            return false;
        }
        r_inverse(index, index) = 1.0 / weight;
    }

    Eigen::MatrixXd hamiltonian =
        Eigen::MatrixXd::Zero(2 * state_size, 2 * state_size);
    hamiltonian.block(0, 0, state_size, state_size) = a_;
    hamiltonian.block(0, state_size, state_size, state_size) =
        -b_ * r_inverse * b_.transpose();
    hamiltonian.block(state_size, 0, state_size, state_size) = -q;
    hamiltonian.block(state_size, state_size, state_size, state_size) =
        -a_.transpose();

    Eigen::ComplexEigenSolver<Eigen::MatrixXd> eigen_solver(hamiltonian);
    if (eigen_solver.info() != Eigen::Success) {
        error = "Hamiltonian eigen decomposition failed";
        return false;
    }

    const auto eigenvalues = eigen_solver.eigenvalues();
    const auto eigenvectors = eigen_solver.eigenvectors();
    std::vector<Eigen::Index> stable_indices;
    stable_indices.reserve(static_cast<std::size_t>(state_size));
    for (Eigen::Index index = 0; index < eigenvalues.size(); ++index) {
        if (eigenvalues[index].real() < -1.0e-8) {
            stable_indices.push_back(index);
        }
    }

    if (static_cast<Eigen::Index>(stable_indices.size()) != state_size) {
        error = "Riccati Hamiltonian did not provide the required stable subspace";
        return false;
    }

    Eigen::MatrixXcd u1(state_size, state_size);
    Eigen::MatrixXcd u2(state_size, state_size);
    for (Eigen::Index column = 0; column < state_size; ++column) {
        u1.col(column) =
            eigenvectors.block(0, stable_indices[static_cast<std::size_t>(column)],
                               state_size, 1);
        u2.col(column) =
            eigenvectors.block(state_size,
                               stable_indices[static_cast<std::size_t>(column)],
                               state_size, 1);
    }

    const auto u1_decomposition = u1.fullPivLu();
    if (!u1_decomposition.isInvertible()) {
        error = "Riccati stable subspace is singular";
        return false;
    }

    const Eigen::MatrixXcd p_complex = u2 * u1.inverse();
    const double max_imaginary = p_complex.imag().cwiseAbs().maxCoeff();
    if (!std::isfinite(max_imaginary) || max_imaginary > 1.0e-5) {
        error = "Riccati solution has a significant imaginary component";
        return false;
    }

    Eigen::MatrixXd p = p_complex.real();
    p = 0.5 * (p + p.transpose());

    gain = r_inverse * b_.transpose() * p;
    if (!gain.allFinite()) {
        error = "Computed LQR gain contains a non-finite value";
        gain.resize(0, 0);
        return false;
    }

    const Eigen::MatrixXd residual =
        a_.transpose() * p + p * a_ -
        p * b_ * r_inverse * b_.transpose() * p + q;
    const double residual_norm = residual.norm();
    const double residual_scale =
        1.0 + q.norm() + a_.norm() * p.norm() +
        (p * b_ * r_inverse * b_.transpose() * p).norm();
    if (!std::isfinite(residual_norm) || !std::isfinite(residual_scale) ||
        residual_norm > 1.0e-6 * residual_scale) {
        error = "Riccati residual check failed";
        gain.resize(0, 0);
        return false;
    }

    return true;
}
