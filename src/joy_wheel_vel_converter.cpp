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
    
}