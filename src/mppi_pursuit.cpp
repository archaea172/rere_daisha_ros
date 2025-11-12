#include "mppi.hpp"
#include <iostream>

int main(int argc, char *argv[])
{
    MPPIControler debug(3, 2);
    auto matrix = debug.sampling_dim2();
    std::cout << matrix << std::endl;
    Eigen::VectorXd init(3);
    init << 0, 0, 0;
    std::cout << debug.predict(init, matrix) << std::endl;;
}