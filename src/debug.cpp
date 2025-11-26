#include <vector>
#include <cmath>
#include <random>
#include <opencv2/opencv.hpp>
#include <iostream>

#include <chrono>

#include "mppi.hpp"

int main()
{
    
    // Eigen::MatrixXd sig(2, 2);
    // sig << 0.5, 0,
    // 0, 1.5;
    // Eigen::VectorXd weight_vector(3);//pos, input, smooth
    // weight_vector << 10, 0.01, 0.1;
    // auto debug = MPPIControler(50, 2, 800, 0.03, sig, 0.5, 0.19, weight_vector, 0.8);
    // Eigen::VectorXd state(3);
    // state << 0, 0, 0;
    // Eigen::VectorXd pos_ref(2);
    // pos_ref << 1, 0;

    // auto start = std::chrono::high_resolution_clock::now();
    // auto input = debug.run(state, pos_ref);
    // auto end = std::chrono::high_resolution_clock::now();

    // std::cout << input << std::endl;
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    // std::cout << duration.count() << std::endl;

    return 0;
}