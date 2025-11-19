#include "yasarobo_bridge.hpp"

Ros2CanBridge::Ros2CanBridge()
: rclcpp_lifecycle::LifecycleNode(std::string("ros2_can_bridge")),
Ifname("can0")
{

}

Ros2CanBridge::CallbackReturn Ros2CanBridge::on_configure(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(
      get_logger(),
      "on_configure() called. state: id=%u, label=%s",
      state.id(),
      state.label().c_str());
    return CallbackReturn::SUCCESS;
}