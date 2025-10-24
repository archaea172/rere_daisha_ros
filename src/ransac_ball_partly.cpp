#include "ransac_ball_partly.hpp"

RansacBallPartlyNode::RansacBallPartlyNode()
{
    /*parameter declare begin*/
    this->declare_parameter<double>("ball_r", 0.04);
    this->declare_parameter<int>("max_loop", 100);
    this->declare_parameter<double>("threshold", 0.2);
    this->declare_parameter<int>("min_samples", 40);
    /*parameter declare end*/

    /*parameter set begin*/
    this->ball_r = (float)this->get_parameter("ball_r").as_double();
    this->max_loop = this->get_parameter("max_loop").as_int();
    this->threshold = (float)this->get_parameter("threshold").as_double();
    this->min_samples = this->get_parameter("min_samples").as_int();
    /*parameter set end*/

    /*ransac initialize begin*/
    ransac_ball = std::make_unique<RansacBall>(
        this->ball_r,
        this->max_loop,
        this->threshold,
        this->min_samples
    );
    /*ransac initialize end*/

    /*node func initialize begin*/
    this->nearest_ball_position = this->create_publisher<rere_daisha_msgs::msg::BallPosition>(
        std::string("nearest_ball_position"),
        rclcpp::SystemDefaultsQoS()
    );
    
    this->parameter_callback_hanle_ = this->add_on_set_parameters_callback(
        std::bind(&RansacBallNode::parameters_callback, this, _1)
    );
    /*node func initialize end*/
}

RansacBallPartlyNode::CallbackReturn RansacBallPartlyNode::on_configure(const rclcpp_lifecycle::State &state)
{
    return CallbackReturn::SUCCESS;
}

RansacBallPartlyNode::CallbackReturn RansacBallPartlyNode::on_activate(const rclcpp_lifecycle::State &state)
{
    /*node func begin*/
    this->nearest_ball_position->on_activate();
    
    this->lidar_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
        std::string("scan"),
        rclcpp::SystemDefaultsQoS(),
        std::bind(&RansacBallPartlyNode::lidar_callback, this, _1)
    );

    this->ball_yolo_subscriber = this->create_subscription<rere_daisha_msgs::msg::BallPositionArray>(
        std::string("ball_position_yolo"),
        rclcpp::SystemDefaultsQoS(),
        std::bind(&RansacBallPartlyNode::ball_callback, this, _1
    );
    /*node func end*/

    return CallbackReturn::SUCCESS;
}

RansacBallPartlyNode::CallbackReturn RansacBallPartlyNode::on_deactivate(const rclcpp_lifecycle::State &state)
{
    this->nearest_ball_position->on_deactivate();
    this->lidar_subscriber.reset();
    this->ball_yolo_subscriber.reset();
    return CallbackReturn::SUCCESS;
}

RansacBallPartlyNode::CallbackReturn RansacBallPartlyNode::on_cleanup(const rclcpp_lifecycle::State &state)
{
    this->lidar_subscriber.reset();
    this->ball_yolo_subscriber.reset();
    return CallbackReturn::SUCCESS;
}

RansacBallPartlyNode::CallbackReturn RansacBallPartlyNode::on_error(const rclcpp_lifecycle::State &state)
{
    return CallbackReturn::SUCCESS;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<RansacBallNode> node = std::make_shared<RansacBallPartlyNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}