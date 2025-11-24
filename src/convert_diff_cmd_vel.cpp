#include "convert_diff_cmd_vel.hpp"

ConvertDiffCmdVel::ConvertDiffCmdVel()
: rclcpp::Node(std::string("convert_diff_cmd_vel")),
msg_flag_(false),
watch_dog_time_(rclcpp::Duration::from_seconds(0.5)),
last_msg_time_(this->now())
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

    this->pub_timer = this->create_wall_timer(
        0.05s,
        std::bind(&ConvertDiffCmdVel::timer_callback, this)
    );
    
    this->parameter_callback_hanle_ = this->add_on_set_parameters_callback(
        std::bind(&ConvertDiffCmdVel::parameters_callback, this, _1)
    );
}

void ConvertDiffCmdVel::wheel_vel_callback(const std_msgs::msg::Float64MultiArray::SharedPtr rxdata)
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
        txdata.linear.set__y(0);
        txdata.angular.set__z(0);
    }
    else
    {
        double vr = vel_.data[0];
        double vl = vel_.data[1];
        double v = (vr + vl) / 2;
        double omega = (vr - vl) / (2 * this->wheel_distance);

        txdata.linear.set__y(v);
        txdata.angular.set__z(omega);
    }

    this->cmd_vel_publisher->publish(txdata);
}

rcl_interfaces::msg::SetParametersResult ConvertDiffCmdVel::parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters
)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    for (const auto &param : parameters)
    {
        if (param.get_name() == "wheel_distance" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
            this->wheel_distance = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Parameter changed");
        }
        else
        {
            result.successful = false;
            result.reason = "fail!";
            RCLCPP_INFO(this->get_logger(), "Parameter change fail!!");
        }
    }
    return result;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<ConvertDiffCmdVel> node = std::make_shared<ConvertDiffCmdVel>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}