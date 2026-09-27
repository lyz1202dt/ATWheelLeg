#pragma once


//低通滤波器
class LowPassFilter {
public:
    explicit LowPassFilter(double alpha = 0.5);
    double update(const double &raw, double filter_alpha);
    void reset(const double &value=0.0);
    double alpha{0.5};
private:
    double filtered_value_{0.0};
    bool initialized_{false};
};



