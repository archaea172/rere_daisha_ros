#include "mppi.hpp"
#include <iostream>

int main(int argc, char *argv[])
{
    MPPIControler debug(10, 2);
    auto matrix = debug.sampling_dim2();
    std::cout << matrix << std::endl;
    Eigen::VectorXd init(3);
    init << 0, 0, 0;
    auto state = debug.predict(init, matrix);
    std::cout << state << std::endl;;

    auto smooth = debug.vel_smooth(state.row(0).transpose());
    std::cout << smooth << std::endl;;
}