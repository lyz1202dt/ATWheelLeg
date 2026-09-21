#pragma once

#include <Eigen/Dense>

class IMUBase {
public:
    IMUBase();
    virtual void init() { return; }
    virtual bool is_ready() { return true; }
    virtual bool update(Eigen::Quation& q, Eigen::Vector3d& angular, Eigen::Vector3d& acc, const float& dt) { return true; }
};
