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

    void set_ball_r(float new_ball_r);
    void set_max_loop(int new_max_loop);
    void set_threshold(float threshold);
    void set_min_samples(int new_min_samples);

private:
    std::vector<int> sampring(uint max_val, uint num);

    float ball_r;
    int max_loop;
    float threshold;
    int min_samples;
};