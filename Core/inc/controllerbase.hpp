#pragma once

#include <Eigen/Dense>

template<int In, int Out>
class ControllerBase {
public:
    using InputVector = Eigen::Vector<float, In>;
    using OutputVector = Eigen::Vector<float, Out>;

    ControllerBase() = default;
    virtual ~ControllerBase() = default;

    virtual bool update(InputVector& measure, OutputVector& output) = 0;
    virtual bool reset() = 0;
};
