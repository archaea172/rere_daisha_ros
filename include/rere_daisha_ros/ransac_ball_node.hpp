#pragma once

#include "ransac_ball.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class RansacBallNode : public rclcpp_lifecycle::LifecycleNode
{

};