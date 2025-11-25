#include "mppi.hpp"

MPPIControler::MPPIControler(
    int horizon_step,
    int dim_num,
    int loop_num,
    double dt_,
    Eigen::MatrixXd sig_,
    double max_wheel_vel,
    double wheel_distance,
    Eigen::VectorXd weight_array,
    double lambda_)
: horizon_step_(horizon_step), dim_num_(dim_num), loop_num_(loop_num),
sig(sig_), gen(1234), uni(0.0, 1.0), clamp_abs(max_wheel_vel),
d(wheel_distance), dt(dt_),
weight_vector(weight_array), lambda(lambda_)
{
    this->mu.resize(2);
    this->mu << 0, 0;

    this->v_ref = this->clamp_abs;
    this->omega_ref = this->clamp_abs / this->d;

    Eigen::LLT<Eigen::MatrixXd> llt(this->sig);
    if (llt.info() == Eigen::Success) this->error_L_matrix = llt.matrixL();
    else this->error_L_matrix = Eigen::MatrixXd::Identity(2, 2);
}

Eigen::VectorXd MPPIControler::run(const Eigen::VectorXd &state, const nav_msgs::msg::Path& path)
{
    Eigen::VectorXd evaluation_result(this->loop_num_);
    Eigen::MatrixXd input_first(2, this->loop_num_);

    Eigen::MatrixXd path_ref = this->pathToEigenMatrix(path, state(0), state(1));
    const auto& goal_pos = path.poses.back().pose.position;
    Eigen::VectorXd goal_pos_eigen(2);
    goal_pos_eigen << goal_pos.x, goal_pos.y;

    #pragma omp parallel for
    for (size_t i = 0; i < (size_t)this->loop_num_; i++)
    {
        Eigen::MatrixXd input_sample = this->sampling_dim2();
        Eigen::MatrixXd state_sample = this->predict(state, input_sample);
        evaluation_result(i) = evaluation(state_sample, input_sample, path_ref, goal_pos_eigen);
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
    thread_local std::mt19937_64 local_gen{1234u + static_cast<unsigned>(omp_get_thread_num())};
    std::normal_distribution<double> dist(0.0, 1.0);
    
    Eigen::MatrixXd y_s(this->dim_num_, this->horizon_step_);
    for (size_t i = 0; i < (size_t)this->horizon_step_; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            y_s(j, i) = dist(local_gen);
        }
    }

    Eigen::MatrixXd z_s = this->error_L_matrix * y_s;
    z_s.colwise() += this->mu;

    z_s = z_s.cwiseMax(-this->clamp_abs).cwiseMin(clamp_abs);

    return z_s;
}

Eigen::MatrixXd MPPIControler::predict(const Eigen::VectorXd &state_init, const Eigen::MatrixXd &input_matrix)
{
    Eigen::RowVectorXd v_vector = input_matrix.colwise().sum()/2;
    Eigen::RowVectorXd omega_vector = (input_matrix.row(0) - input_matrix.row(1)) / (2*d);
    Eigen::MatrixXd state_vector(state_init.size(), this->horizon_step_ + 1);

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

double MPPIControler::evaluation(const Eigen::MatrixXd &state_array, const Eigen::MatrixXd &input_state, const Eigen::MatrixXd &path_ref, const Eigen::VectorXd &goal_pose)
{
    Eigen::VectorXd evaluation_result(4);
    evaluation_result << this->path_error(state_array, path_ref), this->input_error(input_state), this->input_smooth(input_state), this->pos_error(state_array, goal_pose);
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
    Eigen::ArrayXXd v_avg = input_State.colwise().sum().array() / 2.0;
    double result = (v_avg - this->v_ref).square().sum();
    return result;
}

double MPPIControler::path_error(const Eigen::MatrixXd &input_State, const Eigen::MatrixXd &path_ref)
{
    Eigen::MatrixXd predicted_xy = input_State.block(0, 1, 2, this->horizon_step_);
    Eigen::MatrixXd diff = path_ref - predicted_xy;
    double result = diff.array().square().sum();
    return result;
}

void MPPIControler::set_horizon_step(int new_horizon_step)
{
    this->horizon_step_ = new_horizon_step;
}
void MPPIControler::set_loop_num(int new_loop_num)
{
    this->loop_num_ = new_loop_num;
}
void MPPIControler::set_dt(double new_dt)
{
    this->dt = new_dt;
}
void MPPIControler::set_sig(Eigen::MatrixXd new_sig)
{
    this->sig = new_sig;
    Eigen::LLT<Eigen::MatrixXd> llt(this->sig);
    if (llt.info() == Eigen::Success) this->error_L_matrix = llt.matrixL();
    else this->error_L_matrix = Eigen::MatrixXd::Identity(2, 2);
}
void MPPIControler::set_max_wheel_vel(double new_max_wheel_vel)
{
    this->clamp_abs = new_max_wheel_vel;
}
void MPPIControler::set_wheel_distance(double new_wheel_distance)
{
    this->d = new_wheel_distance;
}
void MPPIControler::set_weights(Eigen::VectorXd new_weights)
{
    this->weight_vector = new_weights;
}
void MPPIControler::set_lambda(double new_lambda)
{
    this->lambda = new_lambda;
}

Eigen::MatrixXd MPPIControler::pathToEigenMatrix(const nav_msgs::msg::Path& path, double robot_x, double robot_y)
{
    int closest_idx = 0;
    double min_dist_sq = std::numeric_limits<double>::max();
    for (size_t i = 0; i < path.poses.size(); ++i) {
        double dx = path.poses[i].pose.position.x - robot_x;
        double dy = path.poses[i].pose.position.y - robot_y;
        double dist_sq = dx*dx + dy*dy;
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            closest_idx = i;
        }
    }
    double step_dist = this->v_ref * this->dt;
    std::vector<double> cum_dist;
    cum_dist.push_back(0.0);
    for (size_t i = closest_idx; i < path.poses.size() - 1; ++i) {
        double dx = path.poses[i+1].pose.position.x - path.poses[i].pose.position.x;
        double dy = path.poses[i+1].pose.position.y - path.poses[i].pose.position.y;
        double dist = std::sqrt(dx * dx + dy * dy);
        cum_dist.push_back(cum_dist.back() + dist);
    }
    double total_dist = cum_dist.back();

    int num_steps = static_cast<int>(total_dist / step_dist);
    if (num_steps == 0) num_steps = 1;
    
    int effective_steps = std::min(num_steps, this->horizon_step_);

    Eigen::MatrixXd traj(2, this->horizon_step_);

    const auto& goal_pos = path.poses.back().pose.position;
    const double goal_x = goal_pos.x;
    const double goal_y = goal_pos.y;

    int current_idx = 0;
    for (size_t i = 0; i < this->horizon_step_; i++)
    {
        if (i >= effective_steps) {
            traj(0, i) = goal_x;
            traj(1, i) = goal_y;
            continue;
        }

        double target_d = i * step_dist;

        if (target_d > total_dist) {
            target_d = total_dist;
        }

        while (current_idx < (int)cum_dist.size() - 2 && cum_dist[current_idx + 1] < target_d)
        {
            current_idx++;
        }
        
        double d0 = cum_dist[current_idx];
        double d1 = cum_dist[current_idx + 1];
        double segment_len = d1 - d0;
        
        int p_idx = closest_idx + current_idx;

        double x_out, y_out;

        if (segment_len < 1e-6) {
            x_out = path.poses[p_idx].pose.position.x;
            y_out = path.poses[p_idx].pose.position.y;
        } else {
            double alpha = (target_d - d0) / segment_len;
            if (alpha < 0.0) alpha = 0.0;
            if (alpha > 1.0) alpha = 1.0;
            double x0 = path.poses[p_idx].pose.position.x;
            double y0 = path.poses[p_idx].pose.position.y;
            double x1 = path.poses[p_idx + 1].pose.position.x;
            double y1 = path.poses[p_idx + 1].pose.position.y;

            x_out = (1.0 - alpha) * x0 + alpha * x1;
            y_out = (1.0 - alpha) * y0 + alpha * y1;
        }
        traj(0, i) = x_out;
        traj(1, i) = y_out;
    }
    return traj;
}