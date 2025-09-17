#include "ransac_ball.hpp"

RansacBall::RansacBall(float r, int max_loop, float threshold, int min_samples)
: ball_r(r), max_loop(max_loop), threshold(threshold), min_samples(min_samples)
{

}

std::vector<std::vector<float>> RansacBall::run(std::vector<std::vector<float>> points)
{
    std::vector<std::vector<float>> candidate_center;

    for (size_t i = 0; i < this->max_loop; i++)
    {
        std::vector<int> sampring_index = this->sampring(points.size(), 2);
        std::vector<float> point0 = candidate_center[sampring_index[0]];
        std::vector<float> point1 = candidate_center[sampring_index[1]];

        float dx = point1[0] - point0[0];
        float dy = point1[1] - point0[1];
        float d = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));

        if (d >= 2*this->ball_r);
        else
        {
            float m_x = (point0[0] + point1[0]) / 2;
            float m_y = (point0[1] + point1[1]) / 2;
            float h = std::sqrt(std::pow(this->ball_r, 2) - std::pow(d/2, 2));
            float u_x = -dy/d;
            float u_y =  dx/d;

            std::vector<float> center0(2);
            std::vector<float> center1(2);
            center0[0] = m_x + h*u_x;
            center0[1] = m_y + h*u_y;
            center1[0] = m_x - h*u_x;
            center1[1] = m_y - h*u_y;
        }
    }
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