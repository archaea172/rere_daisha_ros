#pragma once

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

class ConvertDiffCmdVel
: rclcpp::Node
{
public:
    ConvertDiffCmdVel();
private:
    void wheel_vel_callback(const std_msgs::msg::Float64MultiArray::SharedPtr rxdata);
    
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_vel_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
};