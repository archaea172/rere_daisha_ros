#include "ransac_ball.hpp"
#include <vector>
#include <cmath>
#include <random>
#include <opencv2/opencv.hpp>
#include <iostream>

#include <chrono>

std::vector<std::vector<float>> generateCirclePointCloud(float radius, int num_points, float noise_level, int dist) {
    std::vector<std::vector<float>> point_cloud;
    point_cloud.reserve(num_points); // メモリを事前に確保

    // C++11以降のモダンな乱数生成器
    std::random_device rd;
    std::mt19937 gen(rd());
    // 平均0, 標準偏差noise_levelの正規分布
    std::normal_distribution<> d(0, noise_level);

    // M_PIが未定義の場合の対策
    #ifndef M_PI
    const float M_PI = 3.14159265358979323846;
    #endif

    for (int i = 0; i < num_points; ++i) {
        // 0から2πまでの角度を計算
        float theta = 2.0 * M_PI * static_cast<float>(i) / static_cast<double>(num_points);
        
        // 円周上の点の座標を計算
        float perfect_x = radius * std::cos(theta);
        float perfect_y = radius * std::sin(theta);
        
        // ノイズを加えた最終的な座標
        std::vector<float> point = {
            dist + perfect_x + (float)d(gen), // x座標 + ノイズ
            dist + perfect_y + (float)d(gen)  // y座標 + ノイズ
        };
        
        point_cloud.push_back(point);
    }
    
    return point_cloud;
}

int main()
{
    auto test_ransac = RansacBall(50, 100, 15, 10);
    std::vector<std::vector<float>> point_cloud;

    for (int i = 0; i < 4; i++)
    {
        std::vector<std::vector<float>> pointf = generateCirclePointCloud(50, 50, 10, i*150);
        std::copy(pointf.begin(), pointf.end(), std::back_inserter(point_cloud));
    }

    auto start = 
    std::chrono::high_resolution_clock::now();
    std::vector<std::vector<float>> circle_center = test_ransac.run(point_cloud);
    auto end = 
    std::chrono::high_resolution_clock::now();
    auto duration =
    std::chrono::duration_cast<std::chrono::nanoseconds>
    (end - start);
    std::cout << duration.count() << std::endl;

    cv::Mat img = cv::Mat::zeros(500, 500, CV_8UC3);

    // point cloud
    for (size_t i = 0; i < (size_t)point_cloud.size(); i++)
    {
        cv::Point2f point;
        point.x = point_cloud[i][0];
        point.y = point_cloud[i][1];
        cv::Scalar dot_S(0, 255, 255);
        cv::circle(img, point, 2, dot_S, -1, cv::LINE_AA);
    }    
    for (size_t i = 0; i < (size_t)circle_center.size(); i++)
    {
        cv::Point2f point;
        point.x = circle_center[i][0];
        point.y = circle_center[i][1];
        cv::Scalar dot_S(255, 0, 255);
        cv::circle(img, point, 2, dot_S, -1, cv::LINE_AA);
    }
    cv::imshow("img", img);
    cv::waitKey(0);
    return 0;
}