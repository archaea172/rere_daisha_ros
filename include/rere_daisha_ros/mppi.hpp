#pragma once

#include <Eigen/Dense>
#include <random>
#include <cmath>

#include <iostream>

class MPPIControler
{
    public:
        MPPIControler(int sample_num, int dim_num);
    // private:
        Eigen::MatrixXd sampling_dim2();
        Eigen::MatrixXd predict(Eigen::VectorXd state_init, Eigen::MatrixXd iput_matrix);

        int sample_num_;
        int dim_num_;

        Eigen::VectorXd mu;
        Eigen::MatrixXd sig;

        std::mt19937_64 gen;
        std::uniform_real_distribution<double> uni;

        double clamp_abs;
};