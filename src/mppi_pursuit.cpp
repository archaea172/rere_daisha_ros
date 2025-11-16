#include "mppi_pursuit.hpp"

MPPIPursuitNode::MPPIPursuitNode()
: rclcpp_lifecycle::LifecycleNode(std::string("mppi_pursuit_node"))
{

}

MPPIPursuitNode::CallbackReturn MPPIPursuitNode::on_configure(const rclcpp_lifecycle::State &state)
{
    return CallbackReturn::SUCCESS;
}

MPPIPursuitNode::CallbackReturn MPPIPursuitNode::on_activate(const rclcpp_lifecycle::State &state)
{
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