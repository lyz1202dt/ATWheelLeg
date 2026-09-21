#pragma once

#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

using LqrGainMatrix = Eigen::Matrix<double, 2, 6>;
using LqrStateVector = Eigen::Matrix<double, 6, 1>;

class LqrGainScheduler {
public:
    LqrGainScheduler();

    bool load_table(const std::vector<double>& lengths,
                    const std::vector<double>& values,
                    std::string& error);
    bool update(double leg_length);
    bool empty() const;

    const LqrGainMatrix& ground_gain() const;
    const LqrGainMatrix& air_gain() const;

private:
    using TableEntry = std::pair<double, LqrGainMatrix>;

    std::vector<TableEntry> table_;
    LqrGainMatrix ground_gain_;
    LqrGainMatrix air_gain_;
};
