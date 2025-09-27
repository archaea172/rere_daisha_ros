#include "ransac_ball_node.hpp"

RansacBallNode::RansacBallNode()
: rclcpp_lifecycle::LifecycleNode(std::string("ransac_ball_node"))
{
    /*ransac initialize begin*/
    ransac_ball = std::make_unique<RansacBall>(
        this->ball_r,
        this->max_loop,
        this->threshold,
        this->min_samples
    );
    /*ransac initialize end*/

    /*node func initialize begin*/
    this->ball_position_publisher = this->create_publisher<sensor_msgs::msg::PointCloud>(
        std::string("ball_position_ransac"),
        rclcpp::SystemDefaultsQoS()
    );
    
    this->parameter_callback_hanle_ = this->add_on_set_parameters_callback(
        std::bind(&RansacBallNode::parameters_callback, this, _1)
    );
    /*node func initialize end*/
}

int main()
{

}