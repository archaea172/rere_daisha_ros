#include "mppi.hpp"

MPPIControler::MPPIControler(int sample_num, int dim_num)
: sample_num_(sample_num), dim_num_(dim_num),
gen(1234), uni(0.0, 1.0), clamp_abs(0.5)
{
    this->mu.resize(2);
    this->sig.resize(2, 2);
    this->mu << 0, 0;
    this->sig << 1, 0,
                 0, 1;
}

Eigen::MatrixXd MPPIControler::sampling_dim2()
{
    Eigen::MatrixXd y_s(this->dim_num_, this->sample_num_);
    for (int i = 0; i < this->sample_num_; i++)
    {
        double u1 = this->uni(this->gen);
        double u2 = this->uni(this->gen);
        std::max(u1, std::numeric_limits<double>::min());

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

Eigen::MatrixXd MPPIControler::predict(Eigen::VectorXd state_init, Eigen::MatrixXd iput_matrix)
{
    Eigen::RowVectorXd v_vector = iput_matrix.colwise().sum()/2;
    Eigen::RowVectorXd omega_vector = (iput_matrix.row(0) - iput_matrix.row(1)) / 2;

    
}