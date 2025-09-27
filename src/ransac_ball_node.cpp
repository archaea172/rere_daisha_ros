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

RansacBallNode::CallbackReturn RansacBallNode::on_configure(const rclcpp_lifecycle::State &state)
{
    return CallbackReturn::SUCCESS;
}

RansacBallNode::CallbackReturn RansacBallNode::on_activate(const rclcpp_lifecycle::State &state)
{
    /*node func begin*/
    this->ball_position_publisher->on_activate();
    lidar_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
        std::string("scan"),
        rclcpp::SystemDefaultsQoS(),
        std::bind(&RansacBallNode::lidar_callback, this, _1)
    );
    /*node func end*/
    return CallbackReturn::SUCCESS;
}

RansacBallNode::CallbackReturn RansacBallNode::on_deactivate(const rclcpp_lifecycle::State &state)
{
    this->ball_position_publisher->on_deactivate();
    this->lidar_subscriber.reset();
    return CallbackReturn::SUCCESS;
}

RansacBallNode::CallbackReturn RansacBallNode::on_cleanup(const rclcpp_lifecycle::State &state)
{
    this->lidar_subscriber.reset();
    return CallbackReturn::SUCCESS;
}

RansacBallNode::CallbackReturn RansacBallNode::on_error(const rclcpp_lifecycle::State &state)
{
    return CallbackReturn::SUCCESS;
}


RansacBallNode::CallbackReturn RansacBallNode::on_shutdown(const rclcpp_lifecycle::State &state)
{
    this->lidar_subscriber.reset();
    this->ball_position_publisher.reset();
    return CallbackReturn::SUCCESS;
}

void RansacBallNode::lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr rxdata)
{
    std::vector<std::vector<float>> points;
    size_t index = 0;
    for (float angle = rxdata->angle_min; angle <= rxdata->angle_max; angle+=rxdata->angle_increment)
    {
        float x = std::cos(angle)*rxdata->ranges[index];
        float y = std::sin(angle)*rxdata->ranges[index];
        std::vector<float> point = {x, y};
        points.push_back(point);
        index++;
    }
    std::vector<std::vector<float>> ball_centers = this->ransac_ball->run(points);

    sensor_msgs::msg::PointCloud txdata;
    for (size_t i = 0; i < ball_centers.size(); i++)
    {
        geometry_msgs::msg::Point32 point_txdata;
        point_txdata.x = ball_centers[i][0];
        point_txdata.y = ball_centers[i][1];
        point_txdata.z = this->height;
        txdata.points.push_back(point_txdata);
    }
    if (ball_position_publisher->is_activated()) ball_position_publisher->publish(txdata);
}

rcl_interfaces::msg::SetParametersResult RansacBallNode::parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters
)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    return result;
}

int main()
{

}