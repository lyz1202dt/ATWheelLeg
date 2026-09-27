#include "tools/estimater.hpp"

#include <algorithm>
#include <cmath>

LowPassFilter::LowPassFilter(double alpha): alpha(alpha)
{

}

double LowPassFilter::update(const double& input, double filter_alpha) {
    alpha = std::clamp(std::isfinite(filter_alpha) ? filter_alpha : alpha, 0.0, 1.0);
    filtered_value_ = (1.0 - alpha) * filtered_value_ + alpha * input;
    return filtered_value_;
}

void LowPassFilter::reset(const double& value) {
    filtered_value_ = value;
    initialized_ = false;
}
