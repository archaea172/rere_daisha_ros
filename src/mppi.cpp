#include "mppi.hpp"

MPPIControler::MPPIControler(
    int sample_num,
    int dim_num,
    int loop_num,
    double dt_,
    Eigen::MatrixXd sig_,
    double max_wheel_vel,
    double wheel_distance,
    Eigen::VectorXd weight_array,
    double lambda_)
: sample_num_(sample_num), dim_num_(dim_num), loop_num_(loop_num),
sig(sig_), gen(1234), uni(0.0, 1.0), clamp_abs(max_wheel_vel),
d(wheel_distance), dt(dt_),
weight_vector(weight_array), lambda(lambda_)
{
    this->mu.resize(2);
    this->mu << 0, 0;

    this->v_ref = this->clamp_abs;
    this->omega_ref = this->clamp_abs / this->d;
}

Eigen::VectorXd MPPIControler::run(const Eigen::VectorXd &state, const Eigen::VectorXd &pos_ref)
{
    Eigen::VectorXd evaluation_result(this->loop_num_);
    Eigen::MatrixXd input_first(2, this->loop_num_);

    #pragma omp parallel for
    for (size_t i = 0; i < (size_t)this->loop_num_; i++)
    {
        Eigen::MatrixXd input_sample = this->sampling_dim2();
        Eigen::MatrixXd state_sample = this->predict(state, input_sample);
        evaluation_result(i) = evaluation(state_sample, input_sample, pos_ref);
        input_first.col(i) = input_sample.col(0);
    }

    double rho = evaluation_result.minCoeff();
    Eigen::VectorXd weight_result(this->loop_num_);
    weight_result = (Eigen::ArrayXXd(rho - evaluation_result.array()) / lambda).exp().matrix();
    double weight_result_sum = weight_result.sum();
    Eigen::VectorXd input = input_first * weight_result / weight_result_sum;

    this->mu = input;
    return input;
}

Eigen::MatrixXd MPPIControler::sampling_dim2()
{
    Eigen::MatrixXd y_s(this->dim_num_, this->sample_num_);
    for (size_t i = 0; i < (size_t)this->sample_num_; i++)
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

Eigen::MatrixXd MPPIControler::predict(const Eigen::VectorXd &state_init, const Eigen::MatrixXd &input_matrix)
{
    Eigen::RowVectorXd v_vector = input_matrix.colwise().sum()/2;
    Eigen::RowVectorXd omega_vector = (input_matrix.row(0) - input_matrix.row(1)) / (2*d);
    Eigen::MatrixXd state_vector(state_init.size(), this->sample_num_ + 1);

    state_vector.col(0) = state_init;
    for (size_t i = 0; i < (size_t)input_matrix.cols(); i++)
    {
        double vx = v_vector[i]*std::cos(state_vector(2, i));
        double vy = v_vector[i]*std::sin(state_vector(2, i));
        
        Eigen::VectorXd v(3);
        v << vx, vy, omega_vector(i);
        state_vector.col(i + 1) = state_vector.col(i) + v * dt;
    }

    return state_vector;
}

double MPPIControler::evaluation(const Eigen::MatrixXd &state_array, const Eigen::MatrixXd &input_state, const Eigen::VectorXd &pos_ref)
{
    Eigen::VectorXd evaluation_result(3);
    evaluation_result << this->pos_error(state_array, pos_ref), this->input_error(input_state), this->input_smooth(input_state);
    double result = evaluation_result.dot(this->weight_vector);
    return result;
}

double MPPIControler::input_smooth(const Eigen::MatrixXd &input_State)
{
    Eigen::MatrixXd diff = input_State.rightCols(input_State.cols() - 1) - input_State.leftCols(input_State.cols() - 1);
    double result = diff.array().square().sum();

    return result;
}

double MPPIControler::vel_smooth(const Eigen::VectorXd &V)
{
    Eigen::VectorXd diff = V.tail(V.size() - 1) - V.head(V.size() - 1);
    double result = diff.array().square().sum();
    return result;
}

double MPPIControler::pos_error(const Eigen::MatrixXd &input_State, const Eigen::VectorXd &pos_ref)
{
    Eigen::VectorXi rows(2);
    rows << 0, 1;
    double result = (input_State(rows, Eigen::all).colwise() - pos_ref).array().square().sum();
    return result;
}

double MPPIControler::input_error(const Eigen::MatrixXd &input_State)
{
    double result = ((input_State.colwise().sum()/2).array().square() - std::pow(this->v_ref, 2)).sum();
    return result;
}