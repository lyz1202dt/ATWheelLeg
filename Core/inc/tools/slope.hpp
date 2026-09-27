#pragma once

#include <cmath>

template <typename T>
class Slope{
public:
    Slope()=default;
    bool generate_traj(const T &start,const T &stop,float second){
        if (!is_finite(start) || !is_finite(stop) || !std::isfinite(second) || second < 0.0f) {
            generated = false;
            return false;
        }

        this->start = start;
        this->stop = stop;
        traj_second = second;
        generated = true;
        return true;
    }

    bool get_target(T &target,float second){

        if (traj_second < 0.0f || second >= traj_second) {
            target = stop;
            return false;
        }

        if (second < 0.0f) {
            target = start;
            return false;
        }

        const double ratio = static_cast<double>(second) / static_cast<double>(traj_second);
        target = start + (stop - start) * ratio;
        return true;
    }

    T start{};
    T stop{};
    float traj_second{0.0f};
    bool generated{false};

private:
    template <typename U>
    static auto is_finite_impl(const U &value, int) -> decltype(value.allFinite(), bool()){
        return value.allFinite();
    }

    template <typename U>
    static bool is_finite_impl(const U &value, long){
        return std::isfinite(static_cast<double>(value));
    }

    static bool is_finite(const T &value){
        return is_finite_impl(value, 0);
    }
};
