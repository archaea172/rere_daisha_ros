#include "arm_control.hpp"

ArmControl::ArmControl()
: rclcpp::Node(std::string("arm_controller"))
{

}


void ArmControl::arm_control(
    const std::shared_ptr<rere_daisha_msgs::srv::ArmPos::Request> request,
    std::shared_ptr<rere_daisha_msgs::srv::ArmPos::Response> response
)
{

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmControl>());
  rclcpp::shutdown();
  return 0;
}