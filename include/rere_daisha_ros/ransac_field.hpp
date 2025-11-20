#pragma once

#include <Eigen/Dense>
#include <random>

class RansacField
{
public:
    RansacField();
    void run(const Eigen::MatrixXd &scan_points);
private:
    
    Eigen::Vector3d pose_;
    Eigen::Vector3d pose_init_;
};