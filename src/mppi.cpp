#include "mppi.hpp"

MPPIControler::MPPIControler(int sample_num, int dim_num)
: sample_num_(sample_num), dim_num_(dim_num),
gen(1234), uni(0.0, 1.0), clamp_abs(0.5),
d(0.5), loop_num_(1000), dt(0.1)
{
    this->mu.resize(2);
    this->sig.resize(2, 2);
    this->mu << 0, 0;
    this->sig << 1, 0,
                 0, 1;
}

Eigen::VectorXd MPPIControler::run(Eigen::VectorXd state)
{
    Eigen::VectorXd evaluation_result(this->loop_num_);
    Eigen::MatrixXd input_first(2, this->loop_num_);

    for (size_t i = 0; i < (size_t)this->loop_num_; i++)
    {
        Eigen::MatrixXd input_sample = this->sampling_dim2();
        Eigen::MatrixXd state_sample = this->predict(state, input_sample);
        evaluation_result[i] = evaluation(state_sample, input_sample);
        input_first.col(i) = input_sample.col(0);
    }

    double evaluation_result_sum = evaluation_result.sum();
    Eigen::VectorXd input = input_first * evaluation_result / evaluation_result_sum;

    return input;
}

Eigen::MatrixXd MPPIControler::sampling_dim2()
{
    Eigen::MatrixXd y_s(this->dim_num_, this->sample_num_);
    for (int i = 0; i < (size_t)this->sample_num_; i++)
    {
        double u1 = this->uni(this->gen);
        double u2 = this->uni(this->gen);
        u1 = std::max(u1, std::numeric_limits<double>::min());

        double r = std::sqrt(-2.0*std::log(u1));
        double theta = 2.0*M_PI*u2;

        y_s(0, i) = r * std::cos(theta);
        y_s(1, i) = r * std::sin(theta);
    }

    Eigen::LLT<Eigen::MatrixXd> llt(this->sig);
    if (llt.info() != Eigen::Success) {
        Eigen::MatrixXd null_matrix;
        return null_matrix;
    }

    Eigen::MatrixXd P = llt.matrixL();

    Eigen::MatrixXd z_s = P * y_s;
    z_s.colwise() += this->mu;

    z_s = z_s.cwiseMax(-this->clamp_abs).cwiseMin(clamp_abs);

    return z_s;
}

Eigen::MatrixXd MPPIControler::predict(Eigen::VectorXd state_init, Eigen::MatrixXd input_matrix)
{
    Eigen::RowVectorXd v_vector = input_matrix.colwise().sum()/2;
    Eigen::RowVectorXd omega_vector = (input_matrix.row(0) - input_matrix.row(1)) / (2*d);
    Eigen::MatrixXd state_vector(state_init.size(), this->sample_num_ + 1);

    state_vector.col(0) = state_init;
    for (size_t i = 0; i < input_matrix.cols(); i++)
    {
        double vx = v_vector[i]*std::cos(state_vector(2, i));
        double vy = v_vector[i]*std::sin(state_vector(2, i));
        
        Eigen::VectorXd v(3);
        v << vx, vy, omega_vector(i);
        state_vector.col(i + 1) = state_vector.col(i) + v * dt;
    }

    return state_vector;
}

double MPPIControler::evaluation(Eigen::MatrixXd state_array, Eigen::MatrixXd input_state)
{

    double result;
    return result;
}

double MPPIControler::input_smooth(Eigen::MatrixXd input_State)
{
    Eigen::MatrixXd diff = input_State.rightCols(input_State.cols() - 1) - input_State.leftCols(input_State.cols() - 1);
    double result = diff.array().square().sum();
    std::cout << result << std::endl;

    return result;
}