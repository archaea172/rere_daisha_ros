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

Ros2CanBridge::CallbackReturn Ros2CanBridge::on_activate(const rclcpp_lifecycle::State &state)
{
    this->input_subscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        std::string("wheel_vel"),
        rclcpp::SystemDefaultsQoS(),
        std::bind(&Ros2CanBridge::vel_callback, this, _1)
    );

    this->bridge = std::make_unique<CanBridge>(this->Ifname);
    RCLCPP_INFO(
      get_logger(),
      "on_activate() called. state: id=%u, label=%s",
      state.id(),
      state.label().c_str());

    return CallbackReturn::SUCCESS;
}

Ros2CanBridge::CallbackReturn Ros2CanBridge::on_deactivate(const rclcpp_lifecycle::State &state)
{
    this->input_subscriber.reset();
    this->bridge.reset();
    RCLCPP_INFO(
      get_logger(),
      "on_deactivate() called. state: id=%u, label=%s",
      state.id(),
      state.label().c_str());
    return CallbackReturn::SUCCESS;
}

Ros2CanBridge::CallbackReturn Ros2CanBridge::on_cleanup(const rclcpp_lifecycle::State &state)
{
    this->input_subscriber.reset();
    this->bridge.reset();
    RCLCPP_INFO(
      get_logger(),
      "on_cleanup() called. state: id=%u, label=%s",
      state.id(),
      state.label().c_str());
    return CallbackReturn::SUCCESS;
}

Ros2CanBridge::CallbackReturn Ros2CanBridge::on_error(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(
      get_logger(),
      "on_error() called. state: id=%u, label=%s",
      state.id(),
      state.label().c_str());
    return CallbackReturn::SUCCESS;
}

Ros2CanBridge::CallbackReturn Ros2CanBridge::on_shutdown(const rclcpp_lifecycle::State &state)
{
    this->input_subscriber.reset();
    this->bridge.reset();
    RCLCPP_INFO(
      get_logger(),
      "on_shutdown() called. state: id=%u, label=%s",
      state.id(),
      state.label().c_str());
    return CallbackReturn::SUCCESS;
}

void Ros2CanBridge::vel_callback(const std_msgs::msg::Float64MultiArray::SharedPtr rxdata)
{
    int size = rxdata->data.size();
    if (size != 2)
    {
        RCLCPP_INFO(this->get_logger(), "size is not right");
        return;
    }
    std::vector<float> txdata(2);
    txdata[0] = rxdata->data[0];
    txdata[1] = rxdata->data[1];
    this->bridge->send_float(0x200, txdata);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<Ros2CanBridge> node = std::make_shared<Ros2CanBridge>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}