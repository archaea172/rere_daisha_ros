#pragma once

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class ConvertDiffCmdVel
: public rclcpp::Node
{
public:
    ConvertDiffCmdVel();
private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr rxdata);
    void timer_callback();
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
    rclcpp::TimerBase::SharedPtr pub_timer;

    geometry_msgs::msg::Twist vel_;
    rclcpp::Time last_msg_time_;
    bool msg_flag_;
    rclcpp::Duration watch_dog_time_;
};