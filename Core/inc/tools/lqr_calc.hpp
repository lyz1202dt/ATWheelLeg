#pragma once

#include <Eigen/Dense>

#include <string>

class LQRCalc {
public:
    using Matrix = Eigen::MatrixXd;
    using Vector = Eigen::VectorXd;

    LQRCalc(const Matrix& a, const Matrix& b);

    bool set_a(const Matrix& a, std::string& error);
    bool set_b(const Matrix& b, std::string& error);
    bool set_system_matrices(const Matrix& a,
                             const Matrix& b,
                             std::string& error);

    const Matrix& a() const { return a_; }
    const Matrix& b() const { return b_; }

    bool calculate(const Vector& q_diag,
                   const Vector& r_diag,
                   Matrix& gain,
                   std::string& error) const;

private:
    bool validate_system_matrices(const Matrix& a,
                                  const Matrix& b,
                                  std::string& error) const;

    Matrix a_;
    Matrix b_;
};
