#pragma once

#include <vector>
#include <random>
#include <numeric>
#include <algorithm>

class RansacBall
{
public:
    RansacBall(float r, int max_loop, float threshold, int min_samples);
    std::vector<std::vector<float>> run(const std::vector<std::vector<float>> &points);

private:
    std::vector<int> sampring(uint max_val, uint num);

    const float ball_r;
    const int max_loop;
    const float threshold;
    const int min_samples;
};