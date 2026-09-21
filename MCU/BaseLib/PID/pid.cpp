#include "pid.hpp"

PID::PID(float kp, float kd, float ki, float inter_limit, float output_limit, float dt)
    : kp_(kp),
      kd_(kd),
      ki_(ki),
      inter_limit_(inter_limit),
      output_limit_(output_limit),
      dt_(dt),
      last_error_(0.0f),
      last_measure_(0.0f),
      integral_(0.0f) {}

float PID::update(float measure, float target) {
    float error = target - measure;

    integral_ += error * dt_;
    if (integral_ > inter_limit_) {
        integral_ = inter_limit_;
    } else if (integral_ < -inter_limit_) {
        integral_ = -inter_limit_;
    }

    float derivative = (error - last_error_) / dt_;

    float output = kp_ * error + ki_ * integral_ + kd_ * derivative;

    if (output > output_limit_) {
        output = output_limit_;
    } else if (output < -output_limit_) {
        output = -output_limit_;
    }

    last_error_ = error;
    return output;
}

float PID::update_d(float measure, float target) {
    float error = target - measure;

    integral_ += error * dt_;
    if (integral_ > inter_limit_) {
        integral_ = inter_limit_;
    } else if (integral_ < -inter_limit_) {
        integral_ = -inter_limit_;
    }

    float derivative = (last_measure_ - measure) / dt_;

    float output = kp_ * error + ki_ * integral_ + kd_ * derivative;

    if (output > output_limit_) {
        output = output_limit_;
    } else if (output < -output_limit_) {
        output = -output_limit_;
    }

    last_measure_ = measure;
    return output;
}

float PID::update(float measure, float d_measure, float target) {
    float error = target - measure;

    integral_ += error * dt_;
    if (integral_ > inter_limit_) {
        integral_ = inter_limit_;
    } else if (integral_ < -inter_limit_) {
        integral_ = -inter_limit_;
    }

    float output = kp_ * error + ki_ * integral_ - kd_ * d_measure;

    if (output > output_limit_) {
        output = output_limit_;
    } else if (output < -output_limit_) {
        output = -output_limit_;
    }

    return output;
}

float PID::reset() {
    last_error_ = 0.0f;
    last_measure_ = 0.0f;
    integral_ = 0.0f;
    return 0.0f;
}

