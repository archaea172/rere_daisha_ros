#pragma once

#include <vector>
#include <random>
#include <numeric>
#include <algorithm>
#include <unordered_set>

class RansacBall
{
public:
    RansacBall(float r, int max_loop, float threshold, int min_samples);
    std::vector<std::vector<float>> run(const std::vector<std::vector<float>> &points);

private:
    std::vector<int> sampring(uint max_val, uint num);

    float ball_r;
    int max_loop;
    float threshold;
    int min_samples;
};