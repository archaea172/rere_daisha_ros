#include "joy_wheel_vel_converter.hpp"

JoyWheelVelConverter::JoyWheelVelConverter()
: rclcpp::Node(std::string("convert_joy_wheel_vel"))
{
    this->wheel_vel_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "wheel_vel",
        rclcpp::SystemDefaultsQoS()
    );
    this->joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&JoyWheelVelConverter::joy_callback, this, _1)
    );
}

void JoyWheelVelConverter::joy_callback(const sensor_msgs::msg::Joy::SharedPtr rxdata)
{
    std_msgs::msg::Float64MultiArray txdata;
    double v = rxdata->axes[1] * this->max_vel;
    double omega = (rxdata->axes[5] - rxdata->axes[2]) * this->max_omega;

    txdata.data[0] = v + (this->wheel_distance * omega);
    txdata.data[1] = v - (this->wheel_distance * omega);
    this->wheel_vel_publisher_->publish(txdata);
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<JoyWheelVelConverter> node = std::make_shared<JoyWheelVelConverter>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}