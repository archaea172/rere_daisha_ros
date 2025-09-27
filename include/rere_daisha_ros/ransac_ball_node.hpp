#pragma once

#include "ransac_ball.hpp"

#include "cmath"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "geometry_msgs/msg/point.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class RansacBallNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    /*type define begin*/
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    /*type define end*/

    /*class func begin*/
    RansacBallNode();
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
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud>::SharedPtr ball_position_publisher;
    OnSetParametersCallbackHandle::SharedPtr parameter_callback_hanle_;
    /*node value end*/

    /*subscribe callback begin*/
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr rxdata);
    /*subscribe callback end*/
    
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

    float height;
    /*value end*/
};