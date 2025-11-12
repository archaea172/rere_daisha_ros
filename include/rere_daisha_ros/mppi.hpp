#pragma once

#include <Eigen/Dense>
#include <random>
#include <cmath>

class MPPIControler
{
    public:
        MPPIControler(int sample_num, int dim_num);
    private:
        Eigen::MatrixXd sampling_dim2();

        int sample_num_;
        int dim_num_;

        Eigen::VectorXd mu;
        Eigen::MatrixXd sig;

        std::mt19937_64 gen;
        std::uniform_real_distribution<double> uni;
};