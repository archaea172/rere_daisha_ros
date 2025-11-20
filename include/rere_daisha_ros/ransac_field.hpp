#pragma once

#include <Eigen/Dense>
#include <random>

class RansacField
{
public:
    RansacField();
    void run();
private:
    std::vector<int> sampring(uint max_val, uint num);

    Eigen::Vector3d pose_;
    Eigen::Vector3d pose_init_;
};