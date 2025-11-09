#include "ball_converter_ui.hpp"

BallConverterListenerNode::BallConverterListenerNode()
: rclcpp::Node(std::string("ball_converter_listener_node"))
{
    this->ball_yolo_subscriber = this->create_subscription<rere_daisha_msgs::msg::BallPositionArray>(
        std::string("ball_position_yolo"),
        rclcpp::SystemDefaultsQoS(),
        std::bind(&BallConverterListenerNode::yolo_ball_callback, this, _1)
    );
    this->ball_yolo_ui_publisher = this->create_publisher(
        std::string("ball_position_yolo_ui"),
        rclcpp::SystemDefaultsQoS()
    );
    this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void BallConverterListenerNode::yolo_ball_callback(const rere_daisha_msgs::msg::BallPositionArray::SharedPtr rxdata)
{

}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::shutdown();
    return 0;
}