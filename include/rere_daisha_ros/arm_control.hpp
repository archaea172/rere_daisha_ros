#pragma once

#include "rclcpp/rclcpp.hpp"

#include "rere_daisha_msgs/srv/arm_pos.hpp"

class ArmControl
: public rclcpp::Node
{
public:
    ArmControl();
private:
    double theta;
    double r;
    double height;
};