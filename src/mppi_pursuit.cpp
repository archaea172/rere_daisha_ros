#include "mppi_pursuit.hpp"

MPPIPursuitNode::MPPIPursuitNode()
: rclcpp_lifecycle::LifecycleNode(std::string("mppi_pursuit_node"))
{

}

MPPIPursuitNode::CallbackReturn MPPIPursuitNode::on_configure(const rclcpp_lifecycle::State &state)
{

}

MPPIPursuitNode::CallbackReturn MPPIPursuitNode::on_activate(const rclcpp_lifecycle::State &state)
{

}

MPPIPursuitNode::CallbackReturn MPPIPursuitNode::on_deactivate(const rclcpp_lifecycle::State &state)
{
    
}

MPPIPursuitNode::CallbackReturn MPPIPursuitNode::on_cleanup(const rclcpp_lifecycle::State &state)
{
    
}

MPPIPursuitNode::CallbackReturn MPPIPursuitNode::on_error(const rclcpp_lifecycle::State &state)
{
    
}

MPPIPursuitNode::CallbackReturn MPPIPursuitNode::on_shutdown(const rclcpp_lifecycle::State &state)
{
    
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::shutdown();
}