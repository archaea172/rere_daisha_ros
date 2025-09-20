#include "ransac_ball.hpp"

RansacBall::RansacBall(float r, int max_loop, float threshold, int min_samples)
: ball_r(r), max_loop(max_loop), threshold(threshold), min_samples(min_samples)
{

}

std::vector<std::vector<float>> RansacBall::run(const std::vector<std::vector<float>> &points)
{
    if (points.size() < 2) {
        return {};
    }
    std::vector<std::vector<float>> candidate_center;
    std::vector<int> remaining_index(points.size());
    std::iota(remaining_index.begin(), remaining_index.end(), 0);
    int loop_count = 0;

    while (remaining_index.size() >= 2 && loop_count< this->max_loop)
    {
        loop_count++;
        std::vector<int> sampring_index_from_list = this->sampring(remaining_index.size(), 2);

        int original_point_index0 = remaining_index[sampring_index_from_list[0]];
        int original_point_index1 = remaining_index[sampring_index_from_list[1]];

        std::vector<float> point0 = points[original_point_index0];
        std::vector<float> point1 = points[original_point_index1];

        float dx = point1[0] - point0[0];
        float dy = point1[1] - point0[1];
        float d = std::hypot(dx, dy);

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
                std::unordered_set<int> inliers_to_remove(inlier_index_candidate0.begin(), inlier_index_candidate0.end());

                auto it = std::remove_if(remaining_index.begin(), remaining_index.end(),
                [&](int index) {
                    return inliers_to_remove.count(index) > 0;
                });
                remaining_index.erase(it, remaining_index.end());
            }
            if (inlier_index_candidate1.size() > (size_t)this->min_samples) 
            {
                candidate_center.push_back(center1);
                std::unordered_set<int> inliers_to_remove(inlier_index_candidate1.begin(), inlier_index_candidate1.end());

                auto it = std::remove_if(remaining_index.begin(), remaining_index.end(),
                [&](int index) {
                    return inliers_to_remove.count(index) > 0;
                });
                remaining_index.erase(it, remaining_index.end());
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