#include <vector>
#include <cmath>
#include <random>
#include <opencv2/opencv.hpp>
#include <iostream>

#include <chrono>

#include "mppi.hpp"

int main()
{
    
    Eigen::MatrixXd sig(2, 2);
    sig << 1, 0,
    0, 1;
    Eigen::VectorXd weight_vector(3);
    weight_vector << 1, 1, 1;
    auto debug = MPPIControler(1000, 2, 50, 0.5, sig, 0.5, 0.19, weight_vector, 5);
    Eigen::VectorXd state(3);
    state << 0, 0, 0;
    Eigen::VectorXd pos_ref(2);
    pos_ref << 1, 0;

    auto start = std::chrono::high_resolution_clock::now();
    auto input = debug.run(state, pos_ref);
    auto end = std::chrono::high_resolution_clock::now();

    std::cout << input << std::endl;
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << duration.count() << std::endl;

    return 0;
}