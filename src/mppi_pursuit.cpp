#include "mppi.hpp"
#include <iostream>

int main(int argc, char *argv[])
{
    Eigen::MatrixXd sig(2, 2);
    sig << 1, 0,
    0, 1;
    Eigen::VectorXd weight_vector(3);
    weight_vector << 1, 1, 1;
    MPPIControler debug(100, 2, 1000, 0.1, sig, 0.5, 0.5, weight_vector, 5);

    Eigen::VectorXd state(3);
    state << 0, 0, 0;
    Eigen::VectorXd pos_ref(3);
    pos_ref << 1, 0, 0;

    Eigen::VectorXd input = debug.run(state, pos_ref);
    std::cout << input << std::endl;
}