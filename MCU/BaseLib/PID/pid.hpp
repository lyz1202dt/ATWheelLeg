#pragma once

class PID {
public:
    PID(float kp, float kd, float ki, float inter_limit, float output_limit, float dt);
    float update(float measure, float target);                  // 普通PID，对error做差分
    float update_d(float measure, float target);                // 微分先行PID，对measure做差分
    float update(float measure, float d_measure, float target); // 微分先行
    float reset();

private:
    float kp_, kd_, ki_;
    float inter_limit_, output_limit_, dt_;
    float last_error_;
    float last_measure_;
    float integral_;
};