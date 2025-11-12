#include "mppi.hpp"
#include <iostream>

int main(int argc, char *argv[])
{
    MPPIControler debug(100, 2);
    auto matrix = debug.sampling_dim2();

    std::cout << matrix << std::endl;
}