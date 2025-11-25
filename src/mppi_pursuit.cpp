#include "mppi_pursuit.hpp"

MPPIPursuitNode::MPPIPursuitNode()
: rclcpp_lifecycle::LifecycleNode(std::string("mppi_pursuit_node"))
{
    this->dim_num = 2;

    this->declare_parameter<int>("sample_num", 50);
    this->declare_parameter<int>("loop_num", 500);
    this->declare_parameter<double>("dt", 0.5);
    this->declare_parameter<double>("max_wheel_vel", 0.5);
    this->declare_parameter<double>("wheel_distance", 0.5);
    this->declare_parameter<double>("lambda", 5);

    std::vector<double> sig_default = {1, 0, 0, 1};
    this->declare_parameter("sig", sig_default);
    std::vector<double> weight_default = {1, 1, 1};
    this->declare_parameter("weights", weight_default);

    this->sample_num = this->get_parameter("sample_num").as_int();
    this->loop_num = this->get_parameter("loop_num").as_int();
    this->dt = this->get_parameter("dt").as_double();
    this->max_wheel_vel = this->get_parameter("max_wheel_vel").as_double();
    this->wheel_distance = this->get_parameter("wheel_distance").as_double();
    this->lambda_ = this->get_parameter("lambda").as_double();

    std::vector<double> sig_std = this->get_parameter("sig").as_double_array();
    this->sig_.resize(2, 2);
    this->sig_ << 
    sig_std[0], sig_std[1],
    sig_std[2], sig_std[3];
    std::vector<double> weight_std = this->get_parameter("weights").as_double_array();
    this->weight_array = Eigen::Map<const Eigen::VectorXd>(weight_std.data(), weight_std.size());

    this->mppi_controler = std::make_unique<MPPIControler>(
        this->sample_num,
        this->dim_num,
        this->loop_num,
        this->dt,
        this->sig_,
        this->max_wheel_vel,
        this->wheel_distance,
        this->weight_array,
        this->lambda_
    );

    this->wheel_vel_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        std::string("wheel_vel"),
        rclcpp::SystemDefaultsQoS()
    );

    this->parameter_callback_hanle_ = this->add_on_set_parameters_callback(
        std::bind(&MPPIPursuitNode::parameters_callback, this, _1)
    );
}

MPPIPursuitNode::CallbackReturn MPPIPursuitNode::on_configure(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(
      get_logger(),
      "on_configure() called. state: id=%u, label=%s",
      state.id(),
      state.label().c_str());
    return CallbackReturn::SUCCESS;
}

MPPIPursuitNode::CallbackReturn MPPIPursuitNode::on_activate(const rclcpp_lifecycle::State &state)
{
    this->wheel_vel_publisher->on_activate();

    this->pose_subscriber = this->create_subscription<geometry_msgs::msg::Pose2D>(
        std::string("robot_pos"),
        rclcpp::SystemDefaultsQoS(),
        std::bind(&MPPIPursuitNode::pose_callback, this, _1)
    );

    this->pose_ref_subscriber = this->create_subscription<geometry_msgs::msg::Pose2D>(
        std::string("pose_ref"),
        rclcpp::SystemDefaultsQoS(),
        std::bind(&MPPIPursuitNode::pose_ref_callback, this, _1)
    );

    this->mppi_timer = this->create_wall_timer(
        0.1s,
        std::bind(&MPPIPursuitNode::timer_callback, this)
    );
    RCLCPP_INFO(
      get_logger(),
      "on_activate() called. state: id=%u, label=%s",
      state.id(),
      state.label().c_str());

    return CallbackReturn::SUCCESS;
}

MPPIPursuitNode::CallbackReturn MPPIPursuitNode::on_deactivate(const rclcpp_lifecycle::State &state)
{
    this->wheel_vel_publisher->on_deactivate();
    this->pose_subscriber.reset();
    this->pose_ref_subscriber.reset();
    this->mppi_timer.reset();
    RCLCPP_INFO(
      get_logger(),
      "on_deactivate() called. state: id=%u, label=%s",
      state.id(),
      state.label().c_str());
    return CallbackReturn::SUCCESS;
}

MPPIPursuitNode::CallbackReturn MPPIPursuitNode::on_cleanup(const rclcpp_lifecycle::State &state)
{
    this->pose_subscriber.reset();
    this->pose_ref_subscriber.reset();
    this->mppi_timer.reset();
    RCLCPP_INFO(
      get_logger(),
      "on_cleanup() called. state: id=%u, label=%s",
      state.id(),
      state.label().c_str());
    return CallbackReturn::SUCCESS;
}

MPPIPursuitNode::CallbackReturn MPPIPursuitNode::on_error(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(
      get_logger(),
      "on_error() called. state: id=%u, label=%s",
      state.id(),
      state.label().c_str());
    return CallbackReturn::SUCCESS;
}

MPPIPursuitNode::CallbackReturn MPPIPursuitNode::on_shutdown(const rclcpp_lifecycle::State &state)
{
    this->wheel_vel_publisher.reset();
    this->pose_subscriber.reset();
    this->pose_ref_subscriber.reset();
    this->mppi_timer.reset();
    RCLCPP_INFO(
      get_logger(),
      "on_shutdown() called. state: id=%u, label=%s",
      state.id(),
      state.label().c_str());
    return CallbackReturn::SUCCESS;
}

void MPPIPursuitNode::pose_callback(const geometry_msgs::msg::Pose2D::SharedPtr rxdata)
{
    this->pose_ = *rxdata;
    this->pose_flag_ = true;
}

void MPPIPursuitNode::pose_ref_callback(const geometry_msgs::msg::Pose2D::SharedPtr rxdata)
{
    this->pose_ref_ = *rxdata;
    this->pose_ref_flag_ = true;
}

void MPPIPursuitNode::timer_callback()
{
    if (!this->pose_flag_) return;
    if (!this->pose_ref_flag_)
    {
        RCLCPP_INFO(this->get_logger(), "pose_ref is not set");
        return;
    }
    Eigen::VectorXd pose_eigen(3);
    pose_eigen << pose_.x, pose_.y, pose_.theta;
    Eigen::VectorXd pose_ref_eigen(2);
    pose_ref_eigen << pose_ref_.x, pose_ref_.y;
    Eigen::VectorXd input = this->mppi_controler->run(pose_eigen, pose_ref_eigen);
    std::vector<double> input_vector = {input(0), input(1)};

    std_msgs::msg::Float64MultiArray txdata;
    txdata.data = input_vector;
    if (wheel_vel_publisher->is_activated()) wheel_vel_publisher->publish(txdata);
}

rcl_interfaces::msg::SetParametersResult MPPIPursuitNode::parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters
)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (const auto &param : parameters)
    {
        if (param.get_name() == "sample_num" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        {
            this->mppi_controler->set_sample_num(param.as_int());
            RCLCPP_INFO(this->get_logger(), "Parameter changed");
        }
        else if (param.get_name() == "loop_num" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        {
            this->mppi_controler->set_loop_num(param.as_int());
            RCLCPP_INFO(this->get_logger(), "Parameter changed");
        }
        else if (param.get_name() == "dt" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
            this->mppi_controler->set_dt(param.as_double());
            RCLCPP_INFO(this->get_logger(), "Parameter changed");
        }
        else if (param.get_name() == "max_wheel_vel" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
            this->mppi_controler->set_max_wheel_vel(param.as_double());
            RCLCPP_INFO(this->get_logger(), "Parameter changed");
        }
        else if (param.get_name() == "wheel_distance" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
            this->mppi_controler->set_wheel_distance(param.as_double());
            RCLCPP_INFO(this->get_logger(), "Parameter changed");
        }
        else if (param.get_name() == "lambda" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
            this->mppi_controler->set_lambda(param.as_double());
            RCLCPP_INFO(this->get_logger(), "Parameter changed");
        }
        else if (param.get_name() == "sig" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
        {
            std::vector<double> sig_std = param.as_double_array();
            Eigen::MatrixXd sig_eigen(2, 2);
            sig_eigen << 
            sig_std[0], sig_std[1],
            sig_std[2], sig_std[3];
            this->mppi_controler->set_sig(sig_eigen);
            RCLCPP_INFO(this->get_logger(), "Parameter changed");
        }
        else if (param.get_name() == "weights" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
        {
            std::vector<double> weight_std = param.as_double_array();
            Eigen::VectorXd weight_eigen = Eigen::Map<const Eigen::VectorXd>(weight_std.data(), weight_std.size());
            this->mppi_controler->set_weights(weight_eigen);
            RCLCPP_INFO(this->get_logger(), "Parameter changed");
        }
        else
        {
            result.successful = false;
            result.reason = "fail!";
            RCLCPP_INFO(this->get_logger(), "Parameter change fail!!");
        }
    }

    return result;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<MPPIPursuitNode> node = std::make_shared<MPPIPursuitNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}