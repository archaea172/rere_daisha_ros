#pragma once

#include "rclcpp/rclcpp.hpp"

#include "rere_daisha_msgs/srv/arm_pos.hpp"

class ArmControl
: public rclcpp::Node
{
public:
    ArmControl();
private:
    void arm_control(
        const std::shared_ptr<rere_daisha_msgs::srv::ArmPos::Request> request,
        std::shared_ptr<rere_daisha_msgs::srv::ArmPos::Response> response
    );

    double theta;
    double r;
    double height;

    rclcpp::Service<rere_daisha_msgs::srv::ArmPos>::SharedPtr arm_service_;
};