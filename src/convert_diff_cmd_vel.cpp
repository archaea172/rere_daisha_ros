#include "convert_diff_cmd_vel.hpp"

ConvertDiffCmdVel::ConvertDiffCmdVel()
: rclcpp::Node(std::string("convert_diff_cmd_vel"))
{
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

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<ConvertDiffCmdVel> node = std::make_shared<ConvertDiffCmdVel>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}