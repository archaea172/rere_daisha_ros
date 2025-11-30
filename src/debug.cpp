#include <vector>
#include <cmath>
#include <random>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <chrono>

#include "mppi.hpp"

int main(int argc, char *argv[])
{
    
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node(std::string("honban"));
    auto pub_wheel_ = node.create_publisher<std_msgs::msg::Float64MultiArray>(
        std::string("wheel_vel"),
        rclcpp::SystemDefaultsQoS()
    );
    auto pub_arm_ = node.create_publisher<std_msgs::msg::Float64>(
        std::string("base_angle"),
        rclcpp::SystemDefaultsQoS()
    );
    
    std::vector<double> wheel_vel = {0.2, 0.2};
    std_msgs::msg::Float64MultiArray txdata;
    double angle = 0.05;
    std_msgs::msg::Float64 txdata_angle;

    for (size_t i = 0; i < 10; i++)
    {
        txdata.data = wheel_vel;
        pub_wheel_->publish(txdata);
    }
    sleep(1);

    for (size_t i = 0; i < 10; i++)
    {
        wheel_vel = {0, 0};
        txdata.data = wheel_vel;
        pub_wheel_->publish(txdata);
    }

    sleep(2);

    for (size_t i = 0; i < 10; i++)
    {
        angle = 0.5;
        txdata_angle.data = angle;
        pub_arm_->publish(txdata_angle);
    }

    sleep(5);
        

    for (size_t i = 0; i < 10; i++)
    {
        angle = 0;
        txdata_angle.data = angle;
        pub_arm_->publish(txdata_angle);
    }

    rclcpp::shutdown();

    return 0;
}