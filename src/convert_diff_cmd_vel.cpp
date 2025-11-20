#include "convert_diff_cmd_vel.hpp"

ConvertDiffCmdVel::ConvertDiffCmdVel()
: rclcpp::Node(std::string("convert_diff_cmd_vel"))
{
    this->declare_parameter<double>("wheel_distance", 0.5);
    this->wheel_distance = this->get_parameter("wheel_distance").as_double();

    this->cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>(
        std::string("cmd_vel_feedback"),
        rclcpp::SystemDefaultsQoS()
    );
    
    this->wheel_vel_subscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        std::string("wheel_vel"),
        rclcpp::SystemDefaultsQoS(),
        std::bind(&ConvertDiffCmdVel::wheel_vel_callback, this, _1)
    );
}

void ConvertDiffCmdVel::wheel_vel_callback(const std_msgs::msg::Float64MultiArray::SharedPtr rxdata)
{
    double vr = rxdata->data[0];
    double vl = rxdata->data[1];
    double v = (vr + vl) / 2;
    double w = (vr + vl) / 2;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<ConvertDiffCmdVel> node = std::make_shared<ConvertDiffCmdVel>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}