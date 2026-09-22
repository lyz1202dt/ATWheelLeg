#include "lqr_gain_scheduler.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>

namespace {

constexpr size_t kKValuesPerEntry = ::kLqrStateSize * ::kLqrInputSize;

} // namespace

LqrGainScheduler::LqrGainScheduler() {
    ground_gain_.setZero();
    air_gain_.setZero();
}

bool LqrGainScheduler::load_table(const std::vector<double>& lengths, const std::vector<double>& values, std::string& error) {
    if (lengths.empty()) {
        error = "K.lengths must contain at least one leg length";
        return false;
    }
    if (values.size() != lengths.size() * kKValuesPerEntry) {
        error = "K.values must contain exactly 12 values for each K.lengths entry";
        return false;
    }

    table_.clear();
    table_.reserve(lengths.size());
    for (size_t entry_index = 0; entry_index < lengths.size(); ++entry_index) {
        const double length = lengths[entry_index];
        if (!std::isfinite(length)) {
            error = "K.lengths contains a non-finite value";
            table_.clear();
            return false;
        }

        LqrGainMatrix gain;
        for (size_t row = 0; row < kLqrInputSize; ++row) {
            for (size_t column = 0; column < kLqrStateSize; ++column) {
                const size_t value_index = entry_index * kKValuesPerEntry + row * kLqrStateSize + column;
                const double value       = values[value_index];
                if (!std::isfinite(value)) {
                    error = "K.values contains a non-finite value";
                    table_.clear();
                    return false;
                }
                gain(row, column) = value;
            }
        }
        table_.emplace_back(length, gain);
    }

    std::sort(table_.begin(), table_.end(), [](const TableEntry& lhs, const TableEntry& rhs) { return lhs.first < rhs.first; });

    for (size_t index = 1; index < table_.size(); ++index) {
        if (table_[index].first <= table_[index - 1].first) {
            error = "K.lengths entries must be unique";
            table_.clear();
            return false;
        }
    }

    return update(table_.front().first);
}

bool LqrGainScheduler::update(double leg_length) {
    if (!std::isfinite(leg_length)) {
        return false;
    }

    if (!use_k_tab_) {
        return true;
    }

    if (table_.empty()) {
        return false;
    }

    if (leg_length <= table_.front().first || table_.size() == 1U) {
        ground_gain_ = table_.front().second;
    } else if (leg_length >= table_.back().first) {
        ground_gain_ = table_.back().second;
    } else {
        const auto upper = std::lower_bound(
            table_.begin(), table_.end(), leg_length, [](const TableEntry& entry, double length) { return entry.first < length; });
        const auto lower   = std::prev(upper);
        const double ratio = (leg_length - lower->first) / (upper->first - lower->first);
        ground_gain_       = (1.0 - ratio) * lower->second + ratio * upper->second;
    }

    air_gain_.setZero();
    air_gain_(1, 2) = ground_gain_(1, 2);
    air_gain_(1, 3) = ground_gain_(1, 3);
    return true;
}

bool LqrGainScheduler::empty() const { return table_.empty(); }

const LqrGainMatrix& LqrGainScheduler::ground_gain() const { return ground_gain_; }

const LqrGainMatrix& LqrGainScheduler::air_gain() const { return air_gain_; }


bool LqrGainScheduler::solve_lqr_gain(const LqrStateWeight& q_diag, const LqrInputWeight& r_diag, std::string& error) {
    using Matrix6d  = Eigen::Matrix<double, ::kLqrStateSize, ::kLqrStateSize>;
    using Matrix62d = Eigen::Matrix<double, ::kLqrStateSize, ::kLqrInputSize>;
    using Matrix2d  = Eigen::Matrix<double, ::kLqrInputSize, ::kLqrInputSize>;
    using Matrix12d = Eigen::Matrix<double, 2 * ::kLqrStateSize, 2 * ::kLqrStateSize>;
    using Matrix6cd = Eigen::Matrix<std::complex<double>,::kLqrStateSize, ::kLqrStateSize>;

    error.clear();

    Matrix6d A;
    A << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -13.9285, 0.0, 0.6373, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 98.2488, 0.0, 17.6903,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 44.9938, 0.0, 54.3689, 0.0;

    Matrix62d B;
    B << 0.0, 0.0, 15.4595, -3.3942, 0.0, 0.0, -65.5078, 39.5039, 0.0, 0.0, -7.9383, 50.5446;

    Matrix6d Q = Matrix6d::Zero();
    for (std::size_t i = 0; i < ::kLqrStateSize; ++i) {
        if (q_diag[i] < 0.0 || !std::isfinite(q_diag[i])) {
            error = "q_diag values must be finite and greater than or equal to 0";
            return false;
        }
        Q(i, i) = q_diag[i];
    }

    Matrix2d R_inv = Matrix2d::Zero();
    for (std::size_t i = 0; i < ::kLqrInputSize; ++i) {
        if (r_diag[i] <= 0.0 || !std::isfinite(r_diag[i])) {
            error = "r_diag values must be finite and greater than 0";
            return false;
        }
        R_inv(i, i) = 1.0 / r_diag[i];
    }

    Matrix12d H                                                                  = Matrix12d::Zero();
    H.template block<::kLqrStateSize, ::kLqrStateSize>(0, 0)                         = A;
    H.template block<::kLqrStateSize, ::kLqrStateSize>(0, ::kLqrStateSize)             = -B * R_inv * B.transpose();
    H.template block<::kLqrStateSize, ::kLqrStateSize>(::kLqrStateSize, 0)             = -Q;
    H.template block<::kLqrStateSize, ::kLqrStateSize>(::kLqrStateSize, ::kLqrStateSize) = -A.transpose();

    Eigen::ComplexEigenSolver<Matrix12d> eigen_solver(H);
    if (eigen_solver.info() != Eigen::Success) {
        error = "Hamiltonian eigen decomposition failed";
        return false;
    }

    const auto eigenvalues  = eigen_solver.eigenvalues();
    const auto eigenvectors = eigen_solver.eigenvectors();
    std::array<int, ::kLqrStateSize> stable_indices{};
    std::size_t stable_count = 0;
    for (int i = 0; i < eigenvalues.size(); ++i) {
        if (eigenvalues[i].real() < -1.0e-8) {
            if (stable_count >= stable_indices.size()) {
                error = "Riccati Hamiltonian has too many stable eigenvalues";
                return false;
            }
            stable_indices[stable_count++] = i;
        }
    }

    if (stable_count != ::kLqrStateSize) {
        error = "Riccati Hamiltonian did not provide a 6-dimensional stable subspace";
        return false;
    }

    Matrix6cd U1;
    Matrix6cd U2;
    for (std::size_t col = 0; col < ::kLqrStateSize; ++col) {
        U1.col(col) = eigenvectors.template block<::kLqrStateSize, 1>(0, stable_indices[col]);
        U2.col(col) = eigenvectors.template block<::kLqrStateSize, 1>(::kLqrStateSize, stable_indices[col]);
    }

    const auto U1_decomposition = U1.fullPivLu();
    if (!U1_decomposition.isInvertible()) {
        error = "Riccati stable subspace is singular";
        return false;
    }

    const Matrix6cd P_complex = U2 * U1.inverse();
    const double max_imag     = P_complex.imag().cwiseAbs().maxCoeff();
    if (max_imag > 1.0e-5) {
        error = "Riccati solution has a significant imaginary component";
        return false;
    }

    Matrix6d P = P_complex.real();
    P          = 0.5 * (P + P.transpose());

    const LqrGainMatrix gain = R_inv * B.transpose() * P;
    if (!gain.allFinite()) {
        error = "Computed LQR gain contains a non-finite value";
        return false;
    }

    const Matrix6d residual = A.transpose() * P + P * A - P * B * R_inv * B.transpose() * P + Q;
    const double scale      = 1.0 + Q.norm() + A.norm() * P.norm() + (P * B * R_inv * B.transpose() * P).norm();
    if (!std::isfinite(residual.norm()) || residual.norm() > 1.0e-6 * scale) {
        error = "Riccati residual check failed";
        return false;
    }

    // Keep the airborne controller semantics used by LqrGainScheduler:
    // only the pitch and pitch-rate feedback terms remain active in the air.
    ground_gain_ = gain;
    air_gain_.setZero();
    air_gain_(1, 2) = gain(1, 2);
    air_gain_(1, 3) = gain(1, 3);

    return true;
}

void LqrGainScheduler::use_k_tab(bool mode)
{
    use_k_tab_=mode;
}
