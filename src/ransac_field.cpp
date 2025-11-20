#include "ransac_field.hpp"

RansacField::RansacField()
: pose_init_({0, 0, 0})
{

}

void RansacField::run()
{

}

std::vector<int> RansacBall::sampring(uint max_val, uint num)
{
    if (num > max_val) {
        return {};
    }
    std::vector<int> numbers(max_val);
    std::iota(numbers.begin(), numbers.end(), 0);
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(numbers.begin(), numbers.end(), g);
    std::vector<int> return_numbers(numbers.begin(), numbers.begin() + num);
    return return_numbers;
}