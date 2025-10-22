#pragma once

#include "ransac_ball.hpp"

#include "cmath"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "rere_daisha_msgs/msg/ball_position_array.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class RansacBallPartlyNode : public rclcpp_lifecycle::LifecycleNode
{

};