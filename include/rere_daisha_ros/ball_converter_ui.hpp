#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "rere_daisha_msgs/msg/ball_position_array.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class BallConverterListenerNode : public rclcpp::Node
{
private:
    BallConverterListenerNode();
public:
    void yolo_ball_callback(const rere_daisha_msgs::msg::BallPositionArray::SharedPtr rxdata);

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};