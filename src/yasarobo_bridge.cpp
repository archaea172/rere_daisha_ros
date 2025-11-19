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

    this->bridge = std::make_unique<Ros2CanBridge>(this->Ifname);

    return CallbackReturn::SUCCESS;
}

Ros2CanBridge::CallbackReturn Ros2CanBridge::on_activate(const rclcpp_lifecycle::State &state)
{
    this->input_subscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        std::string("wheel_vel"),
        rclcpp::SystemDefaultsQoS(),
        std::bind(&Ros2CanBridge::vel_callback, this, _1)
    );
    RCLCPP_INFO(
      get_logger(),
      "on_activate() called. state: id=%u, label=%s",
      state.id(),
      state.label().c_str());

    return CallbackReturn::SUCCESS;
}