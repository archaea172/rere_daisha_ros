#include "ros2can_bridge.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class Ros2CanBridge
: public rclcpp_lifecycle::LifecycleNode
{
public:
    Ros2CanBridge();
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
private:
    CallbackReturn on_configure(const rclcpp_lifecycle::State &state);
    CallbackReturn on_activate(const rclcpp_lifecycle::State &state);
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state);
    CallbackReturn on_error(const rclcpp_lifecycle::State &state);
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);

    std::unique_ptr<CanBridge> bridge;

    void vel_callback(const std_msgs::msg::Float64MultiArray::SharedPtr rxdata);

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr input_subscriber;
    const char* Ifname;
};