#pragma once

#include <cstdint>


class FSMFactory;

class FSM {
public:
    FSM(char*& fsm_name, void* ctx)
        : fsm_name_(fsm_name)
        , ctx_(ctx) {}

    virtual ~FSM() = default;

    char* get_name() const { return fsm_name_; }

    virtual bool enter(const char* last_state, const uint64_t& time) {
        (void)last_state;
        (void)time;
        return true;
    }

    virtual bool exit(const char* next_state) {
        (void)next_state;
        return true;
    }

    virtual char* check_switch() const {
        return fsm_name_;
    }

    virtual bool run(const uint64_t& time) {
        (void)time;
        return true;
    }

protected:
    char* fsm_name_;
    void* ctx_;

private:
    
};
