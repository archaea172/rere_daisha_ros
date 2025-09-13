#include "ransac_ball.hpp"

RansacBall::RansacBall(float r)
: ball_r(r)
{

}

std::vector<std::vector<float>> RansacBall::run()
{

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