#include "ransac_ball.hpp"

RansacBall::RansacBall(float r, int max_loop, float threshold, int min_samples)
: ball_r(r), max_loop(max_loop), threshold(threshold), min_samples(min_samples)
{

}

std::vector<std::vector<float>> RansacBall::run(std::vector<std::vector<float>> points)
{
    std::vector<std::vector<float>> candidate_center;

}

std::vector<int> RansacBall::sampring(uint max_val, uint num)
{
    if (num > max_val + 1) {
        return {};
    }
    int min_val = 0;
    std::vector<int> numbers(max_val - min_val + 1);
    std::iota(numbers.begin(), numbers.end(), min_val);
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(numbers.begin(), numbers.end(), g);
    std::vector<int> return_numbers(numbers.begin(), numbers.begin() + num);
    return return_numbers;
}