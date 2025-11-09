#include "ball_converter_ui.hpp"

BallConverterListenerNode::BallConverterListenerNode()
: rclcpp::Node(std::string("ball_converter_listener_node"))
{

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