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
public:
    /*type define begin*/
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    /*type define end*/

    /*class func begin*/
    RansacBallPartlyNode();
    /*class func end*/
    
private:
    /*lifecycle callback begin*/
    CallbackReturn on_configure(const rclcpp_lifecycle::State &state);
    CallbackReturn on_activate(const rclcpp_lifecycle::State &state);
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state);
    CallbackReturn on_error(const rclcpp_lifecycle::State &state);
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);
    /*lifecycle callback end*/

    /*node value begin*/
    std::unique_ptr<RansacBall> ransac_ball;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber;
    rclcpp::Subscription<rere_daisha_msgs::msg::BallPositionArray>::SharedPtr ball_yolo_subscriber;
    rclcpp_lifecycle::LifecyclePublisher<rere_daisha_msgs::msg::BallPosition>::SharedPtr nearest_ball_position;
    /*node value end*/

    /*subscriber callback begin*/
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr rxdata);    
    void ball_callback(const rere_daisha_msgs::msg::BallPosition::SharedPtr rxdata);
    /*subscriber callback end*/
    
    /*parameter callback begin*/
    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter> &parameters
    );
    /*parameter callback end*/

    /*value begin*/
    float ball_r;
    int max_loop;
    float threshold;
    int min_samples;
    /*value end*/
};