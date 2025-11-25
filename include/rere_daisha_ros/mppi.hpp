#pragma once

#include <Eigen/Dense>
#include <random>
#include <cmath>

#include <iostream>

#include <omp.h>

#include "nav_msgs/msg/path.hpp"

class MPPIControler
{
    public:
        MPPIControler(
            int horizon_step,
            int dim_num,
            int loop_num,
            double dt,
            Eigen::MatrixXd sig_,
            double max_wheel_vel,
            double wheel_distance,
            Eigen::VectorXd weight_array,
            double lambda_);
        Eigen::VectorXd run(const Eigen::VectorXd &state, const Eigen::VectorXd &pos_ref);
        void set_horizon_step(int horizon_step);
        void set_loop_num(int new_loop_num);
        void set_dt(double new_dt);
        void set_sig(Eigen::MatrixXd new_sig);
        void set_max_wheel_vel(double new_max_wheel_vel);
        void set_wheel_distance(double new_wheel_distance);
        void set_weights(Eigen::VectorXd new_weights);
        void set_lambda(double new_lambda);
    private:
        Eigen::MatrixXd sampling_dim2();
        Eigen::MatrixXd predict(const Eigen::VectorXd &state_init, const Eigen::MatrixXd &iput_matrix);

        double evaluation(const Eigen::MatrixXd &state_array, const Eigen::MatrixXd &input_state, const Eigen::MatrixXd &path_ref);
        double input_smooth(const Eigen::MatrixXd &input_State);
        double vel_smooth(const Eigen::VectorXd &V);
        double pos_error(const Eigen::MatrixXd &input_State, const Eigen::VectorXd &pos_ref);
        double input_error(const Eigen::MatrixXd &input_State);
        double path_error(const Eigen::MatrixXd &input_State, const Eigen::MatrixXd &path_ref);

        int horizon_step_;
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

        Eigen::VectorXd weight_vector;
        double lambda;
        Eigen::MatrixXd error_L_matrix;
};