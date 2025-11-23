#pragma once

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class ConvertDiffCmdVel
: public rclcpp::Node
{
public:
    ConvertDiffCmdVel();
private:
    void wheel_vel_callback(const std_msgs::msg::Float64MultiArray::SharedPtr rxdata);
    void timer_callback();

    OnSetParametersCallbackHandle::SharedPtr parameter_callback_hanle_;
    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter> &parameters
    );

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_vel_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
    rclcpp::TimerBase::SharedPtr pub_timer;

    double wheel_distance;
};