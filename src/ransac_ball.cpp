#include "ransac_ball.hpp"

RansacBall::RansacBall(float r, int max_loop, float threshold, int min_samples)
: ball_r(r), max_loop(max_loop), threshold(threshold), min_samples(min_samples)
{

}

std::vector<std::vector<float>> RansacBall::run(std::vector<std::vector<float>> points)
{
    if (points.size() < 2) {
        return {};
    }
    std::vector<std::vector<float>> candidate_center;
    // std::vector<int> inlier_index;

    for (size_t i = 0; i < (size_t)this->max_loop; i++)
    {
        std::vector<int> sampring_index = this->sampring(points.size(), 2);
        std::vector<float> point0 = points[sampring_index[0]];
        std::vector<float> point1 = points[sampring_index[1]];

        float dx = point1[0] - point0[0];
        float dy = point1[1] - point0[1];
        float d = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));

        if (d < 2*this->ball_r)
        {
            float m_x = (point0[0] + point1[0]) / 2;
            float m_y = (point0[1] + point1[1]) / 2;
            float h = std::sqrt(std::pow(this->ball_r, 2) - std::pow(d/2, 2));
            float u_x = -dy/d;
            float u_y =  dx/d;

            std::vector<float> center0 = {
                m_x + h*u_x,
                m_y + h*u_y
            };
            std::vector<float> center1 = {
                m_x - h*u_x,
                m_y - h*u_y
            };

            std::vector<int> inlier_index_candidate0;
            std::vector<int> inlier_index_candidate1;
            for (size_t j = 0; j < (size_t)points.size(); j++)
            {
                float dist0 = std::hypot(center0[0] - points[j][0], center0[1] - points[j][1]);
                if ((dist0 > this->ball_r - this->threshold) && (dist0 < this->ball_r + this->threshold)) inlier_index_candidate0.push_back(j);
                float dist1 = std::hypot(center1[0] - points[j][0], center1[1] - points[j][1]);
                if ((dist1 > this->ball_r - this->threshold) && (dist1 < this->ball_r + this->threshold)) inlier_index_candidate1.push_back(j);
            }

            if (inlier_index_candidate0.size() > (size_t)this->min_samples) 
            {
                candidate_center.push_back(center0);
                // std::copy(inlier_index_candidate0.begin(), inlier_index_candidate0.end(), std::back_inserter(inlier_index));
            }
            if (inlier_index_candidate1.size() > (size_t)this->min_samples) 
            {
                candidate_center.push_back(center1);
                // std::copy(inlier_index_candidate1.begin(), inlier_index_candidate1.end(), std::back_inserter(inlier_index));
            }
        }
    }
    return candidate_center;
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