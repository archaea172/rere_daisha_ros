#include "ransac_ball.hpp"
#include <vector>
#include <cmath>
#include <random>

std::vector<std::vector<float>> generateCirclePointCloud(float radius, int num_points, float noise_level) {
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
            perfect_x + (float)d(gen), // x座標 + ノイズ
            perfect_y + (float)d(gen)  // y座標 + ノイズ
        };
        
        point_cloud.push_back(point);
    }
    
    return point_cloud;
}

int main()
{
    auto test_ransac = RansacBall(200, 100, 0.3, 10);

    std::vector<std::vector<float>> point_cloud = generateCirclePointCloud(200, 500, 10);
    std::vector<std::vector<float>> circle_center = test_ransac.run(point_cloud);


    return 0;
}