#pragma once

#include <Eigen/Dense>
#include <random>
#include <cmath>

#include <iostream>

class MPPIControler
{
    public:
        MPPIControler(int sample_num, int dim_num);
        Eigen::VectorXd run(Eigen::VectorXd state, Eigen::VectorXd pos_ref);
    // private:
        Eigen::MatrixXd sampling_dim2();
        Eigen::MatrixXd predict(Eigen::VectorXd state_init, Eigen::MatrixXd iput_matrix);

        Eigen::RowVectorXd evaluation(Eigen::MatrixXd state_array, Eigen::MatrixXd input_state);
        double input_smooth(const Eigen::MatrixXd &input_State);
        double vel_smooth(const Eigen::VectorXd &V);
        double pos_error(const Eigen::MatrixXd &input_State);
        double input_error(const Eigen::MatrixXd &input_State);

        int sample_num_;
        int dim_num_;
        int loop_num_;

        Eigen::VectorXd mu;
        Eigen::MatrixXd sig;

        std::mt19937_64 gen;
        std::uniform_real_distribution<double> uni;

        double clamp_abs;

        double d;
        double dt;
        double v_ref;
        double omega_ref;
};