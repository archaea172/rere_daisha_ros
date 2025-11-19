#include "yasarobo_bridge.hpp"

Ros2CanBridge::Ros2CanBridge()
: rclcpp_lifecycle::LifecycleNode(std::string("ros2_can_bridge")),
Ifname("can0")
{

}

