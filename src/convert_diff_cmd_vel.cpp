#include "convert_diff_cmd_vel.hpp"

ConvertDiffCmdVel::ConvertDiffCmdVel()
: rclcpp::Node(std::string("convert_diff_cmd_vel")),
last_msg_time_(this->now()),
msg_flag_(false),
watch_dog_time_(rclcpp::Duration::from_seconds(0.5))
{
    this->cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>(
        std::string("cmd_vel_feedback"),
        rclcpp::SystemDefaultsQoS()
    );

    this->cmd_vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
        std::string("cmd_vel"),
        rclcpp::SystemDefaultsQoS(),
        std::bind(&ConvertDiffCmdVel::cmd_vel_callback, this, _1)
    );

    this->pub_timer = this->create_wall_timer(
        0.05s,
        std::bind(&ConvertDiffCmdVel::timer_callback, this)
    );
}

void ConvertDiffCmdVel::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr rxdata)
{
    this->vel_ = *rxdata;
    this->last_msg_time_ = this->now();
    this->msg_flag_ = true;
}

void ConvertDiffCmdVel::timer_callback()
{
    geometry_msgs::msg::Twist txdata;

    auto now = this->now();
    const bool stale = !this->msg_flag_ || ((now - last_msg_time_) > this->watch_dog_time_);
    if (stale)
    {
        txdata.linear.set__x(0);
        txdata.linear.set__y(0);
        txdata.angular.set__z(0);
    }
    else
    {
        txdata.linear.set__x(this->vel_.linear.x);
        txdata.linear.set__y(0);
        txdata.angular.set__z(this->vel_.angular.z);
    }

    this->cmd_vel_publisher->publish(txdata);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<ConvertDiffCmdVel> node = std::make_shared<ConvertDiffCmdVel>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}