#pragma once

#include <cstdint>
#include <cstring>
#include <memory>
#include <utility>
#include <vector>

#include "fsm.hpp"

class FSMFactory {
public:
    FSMFactory() = default;

    bool register_fsm(std::unique_ptr<FSM> fsm) {
        if (fsm == nullptr) {
            return false;
        }
        if (find_fsm(fsm->get_name()) != nullptr) {
            return false;
        }
        fsm_list_.emplace_back(fsm->get_name(), std::move(fsm));
        return true;
    }

    bool set_init_state(const char* name) {
        if (find_fsm(name) == nullptr) {
            return false;
        }
        current_state_name_ = name;
        return true;
    }

    bool run(const uint64_t& time) {
        FSM* current_fsm = find_fsm(current_state_name_);
        if (current_fsm == nullptr) {
            return false;
        }

        if (first_run) {
            first_run = false;
            current_fsm->enter("", time);
            return true;
        }

        if (state_switch) {
            FSM* next_fsm = find_fsm(next_state_name_);
            if (next_fsm == nullptr) {
                return false;
            }
            if (!current_fsm->exit(next_fsm->get_name())) {
                return false;
            }
            if (!next_fsm->enter(current_fsm->get_name(), time)) {
                return false;
            }
            current_state_name_ = next_state_name_;
            state_switch = false;
        } else {
            if (!current_fsm->run(time)) {
                return false;
            }
            const char* next_state = current_fsm->check_switch();
            if (std::strcmp(next_state, current_fsm->get_name()) != 0) {
                if (find_fsm(next_state) == nullptr) {
                    return false;
                }
                next_state_name_ = next_state;
                state_switch = true;
            }
        }

        return true;
    }

private:
    FSM* find_fsm(const char* name) const {
        for (const auto& entry : fsm_list_) {
            if (std::strcmp(entry.first, name) == 0) {
                return entry.second.get();
            }
        }
        return nullptr;
    }

    bool first_run{true};
    bool state_switch{false};
    const char* next_state_name_{nullptr};
    const char* current_state_name_{nullptr};
    std::vector<std::pair<const char*, std::unique_ptr<FSM>>> fsm_list_;
};
