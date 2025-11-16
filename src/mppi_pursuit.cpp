#include "mppi_pursuit.hpp"

MPPIPursuitNode::MPPIPursuitNode()
: rclcpp_lifecycle::LifecycleNode(std::string("mppi_pursuit_node"))
{

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::shutdown();
}