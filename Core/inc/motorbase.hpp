#pragma once

class Motor {
public:
    explicit Motor(void*) {}
    virtual ~Motor() = default;

    virtual bool init() { return true; }
    virtual bool enable() { return true; }
    virtual bool disable() { return true; }
    virtual int has_error() { return 0; }
    virtual bool clear_error(int cmd) { return true; }
    virtual bool set_command(float pos, float vel, float tor, float kp, float kd) { return true; }
    virtual bool read_state() { return true; } // 请求获取电机状态

    struct {
        int r;
        float rad;
        float continue_rad;
        float vel;
        float toqeue;
    } state;
};
