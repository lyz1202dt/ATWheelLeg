#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

using LqrGainMatrix = Eigen::Matrix<double, 2, 6>;
using LqrStateVector = Eigen::Matrix<double, 6, 1>;
static constexpr std::size_t kLqrStateSize = 6U;
static constexpr std::size_t kLqrInputSize = 2U;
using LqrStateWeight = std::array<double, kLqrStateSize>;
using LqrInputWeight = std::array<double, kLqrInputSize>;

class LqrGainScheduler {
public:
    LqrGainScheduler();

    bool load_table(const std::vector<double>& lengths, const std::vector<double>& values, std::string& error);
    bool update(double leg_length);
    bool empty() const;

    void use_k_tab(bool mode);
    bool solve_lqr_gain(const LqrStateWeight& q_diag, const LqrInputWeight& r_diag, std::string& error);

    const LqrGainMatrix& ground_gain() const;
    const LqrGainMatrix& air_gain() const;

private:
    using TableEntry = std::pair<double, LqrGainMatrix>;

    bool use_k_tab_{true};
    std::vector<TableEntry> table_;
    LqrGainMatrix ground_gain_;
    LqrGainMatrix air_gain_;
};
