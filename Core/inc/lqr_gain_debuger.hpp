#pragma once

#include <Eigen/Dense>
#include <array>
#include <cstddef>
#include <string>

class LqrGainDebuger {
public:
    static constexpr std::size_t kLqrStateSize = 6U;
    static constexpr std::size_t kLqrInputSize = 2U;

    using GainMatrix = Eigen::Matrix<double, kLqrInputSize, kLqrStateSize>;
    using StateWeight = std::array<double, kLqrStateSize>;
    using InputWeight = std::array<double, kLqrInputSize>;

    LqrGainDebuger();
    void bind_k_mat(GainMatrix* k_mat, GainMatrix* air_k_mat);
    bool solve_lqr_gain(const StateWeight& q_diag,
                        const InputWeight& r_diag,
                        std::string& error);

private:
    GainMatrix* k_mat_ = nullptr;
    GainMatrix* air_k_mat_ = nullptr;
};
