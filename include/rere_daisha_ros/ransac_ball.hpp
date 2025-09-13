#pragma once

#include <Eigen/Dense>
#include <vector>
#include <random>
#include <numeric>
#include <algorithm>

class RansacBall
{
public:
    RansacBall(float r);
    std::vector<std::vector<float>> run(std::vector<std::vector<float>> points);

private:
    std::vector<int> sampring(uint max_val, uint num);

    const float ball_r;
};