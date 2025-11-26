#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using std::placeholders::_1;

using namespace std::chrono_literals;

class JoyWheelVelConverter
: public rclcpp::Node
{
public:
    JoyWheelVelConverter();

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr rxdata);

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_vel_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
};