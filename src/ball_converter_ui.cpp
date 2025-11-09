#include "ball_converter_ui.hpp"

BallConverterListenerNode::BallConverterListenerNode()
: rclcpp::Node(std::string("ball_converter_listener_node"))
{
    this->ball_yolo_subscriber = this->create_subscription<rere_daisha_msgs::msg::BallPositionArray>(
        std::string("ball_position_yolo"),
        rclcpp::SystemDefaultsQoS(),
        std::bind(&BallConverterListenerNode::yolo_ball_callback, this, _1)
    );
    this->ball_yolo_ui_publisher = this->create_publisher<rere_daisha_msgs::msg::BallPositionArray>(
        std::string("ball_position_yolo_ui"),
        rclcpp::SystemDefaultsQoS()
    );
    this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void BallConverterListenerNode::yolo_ball_callback(const rere_daisha_msgs::msg::BallPositionArray::SharedPtr rxdata)
{
    std::string source_frame = rxdata->header.frame_id;
    std::string target_frame = "base_link";

    geometry_msgs::msg::TransformStamped transform_stamped;
    try
    {
        transform_stamped = tf_buffer_->lookupTransform(
            target_frame,
            source_frame,
            tf2::TimePointZero
        );
    }
    catch(const tf2::TransformException & ex)
    {
        RCLCPP_WARN(this->get_logger(), "フレームを見つけられません: %s", ex.what());
    }

    rere_daisha_msgs::msg::BallPositionArray txdata = *rxdata;

    try
    {
        for (size_t i = 0; i < rxdata->balls.size(); i++)
        {
            geometry_msgs::msg::Point point_in_target;
            tf2::doTransform(rxdata->balls[i].position, point_in_target, transform_stamped);
            txdata.balls[i].position = point_in_target;
        }
    }
    catch(const tf2::TransformException & ex)
    {
        RCLCPP_WARN(this->get_logger(), "TF変換失敗: %s", ex.what());
    }
    
    this->ball_yolo_ui_publisher.publish(txdata);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<BallConverterListenerNode> node = std::make_shared<BallConverterListenerNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}