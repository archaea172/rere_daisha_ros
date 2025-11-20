#pragma once

#include <Eigen/Dense>

class RansacField
{
public:
    RansacField();
    void run();
private:
    std::vector<int> sampring(uint max_val, uint num);

    Eigen::Vector3d pose;
};