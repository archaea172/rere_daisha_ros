#pragma once

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"

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