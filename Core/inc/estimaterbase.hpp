#pragma once

#include <Eigen/Dense>

template<int In, int Out>
class EstimaterBase {
public:
    using InputVector = Eigen::Vector<float, In>;
    using OutputVector = Eigen::Vector<float, Out>;

    EstimaterBase() = default;
    virtual ~EstimaterBase() = default;

    virtual bool update(InputVector& measure, OutputVector& output) = 0;
    virtual bool reset(OutputVector& output) = 0;
};
