#include "mppi_pursuit.hpp"

MPPIPursuitNode::MPPIPursuitNode()
: rclcpp_lifecycle::LifecycleNode(std::string("mppi_pursuit_node"))
{
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

    this->mppi_timer = this->create_wall_timer(
        0.1s,
        std::bind(&MPPIPursuitNode::timer_callback, this, _1)
    );

    return CallbackReturn::SUCCESS;
}

MPPIPursuitNode::CallbackReturn MPPIPursuitNode::on_deactivate(const rclcpp_lifecycle::State &state)
{
    return CallbackReturn::SUCCESS;
}

MPPIPursuitNode::CallbackReturn MPPIPursuitNode::on_cleanup(const rclcpp_lifecycle::State &state)
{
    return CallbackReturn::SUCCESS;
}

MPPIPursuitNode::CallbackReturn MPPIPursuitNode::on_error(const rclcpp_lifecycle::State &state)
{
    return CallbackReturn::SUCCESS;
}

MPPIPursuitNode::CallbackReturn MPPIPursuitNode::on_shutdown(const rclcpp_lifecycle::State &state)
{
    return CallbackReturn::SUCCESS;
}

void MPPIPursuitNode::pose_callback(const geometry_msgs::msg::Pose2D::SharedPtr rxdata)
{

}

void MPPIPursuitNode::timer_callback()
{
    
}

rcl_interfaces::msg::SetParametersResult MPPIPursuitNode::parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters
)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

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