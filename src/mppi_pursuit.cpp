#include "mppi.hpp"
#include <iostream>

int main(int argc, char *argv[])
{
    MPPIControler debug(3, 2);
    auto matrix = debug.sampling_dim2();
    Eigen::VectorXd init;
    debug.predict(init, matrix);

    std::cout << matrix << std::endl;
}