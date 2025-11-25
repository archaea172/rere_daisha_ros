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
    auto debug = MPPIControler(50, 2, 500, 0.5, sig, 0.5, 0.19, weight_vector, 5);
    Eigen::VectorXd state(3);
    state << 0, 0, 0;
    Eigen::VectorXd pos_ref(2);
    pos_ref << 1, 0;

    auto input = debug.run(state, pos_ref);

    std::cout << input << std::endl;

    return 0;
}