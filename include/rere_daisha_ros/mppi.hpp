#pragma once

#include <Eigen/Dense>
#include <random>
#include <cmath>

class MPPIControler
{
    public:
        MPPIControler(int sample_num, int dim_num);
    private:
        int sample_num_;
        int dim_num_;
};