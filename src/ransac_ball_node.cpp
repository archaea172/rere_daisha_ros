#include "ransac_ball_node.hpp"

RansacBallNode::RansacBallNode()
: rclcpp_lifecycle::LifecycleNode(std::string("ransac_ball_node"))
{
    /*parameter declare begin*/
    this->declare_parameter<double>("ball_r", 0.04);
    this->declare_parameter<double>("max_loop", 100);
    this->declare_parameter<double>("threshold", 0.2);
    this->declare_parameter<double>("min_samples", 40);
    /*parameter declare end*/

    /*parameter set begin*/
    this->ball_r = (float)this->get_parameter("ball_r").as_double();
    this->max_loop = (float)this->get_parameter("max_loop").as_double();
    this->threshold = (float)this->get_parameter("threshold").as_double();
    this->min_samples = (float)this->get_parameter("min_samples").as_double();
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

    for (const auto &param : parameters)
    {
        if (param.get_name() == "ball_r")
        {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                if (param.as_double() > 0.0)
                {
                    this->ransac_ball->set_ball_r(param.as_double());
                    RCLCPP_INFO(this->get_logger(), "Parameter 'ball_radius' changed to: %f", param.as_double());
                }
                else
                {
                    result.successful = false;
                    result.reason = "ball_radius must be positive.";
                }
            }
            else
            {
                result.successful = false;
                result.reason = "Invalid type for ball_radius";
            }
        }
        else if (param.get_name() == "max_loop")
        {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                this->ransac_ball->set_max_loop(param.as_int());
                RCLCPP_INFO(this->get_logger(), "Parameter 'max_loop' changed to: %ld", param.as_int());
            }
            else
            {
                result.successful = false;
                result.reason = "Invalid type for parameter 'max_loop'.";
            }
        }
        else if (param.get_name() == "threshold")
        {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                this->ransac_ball->set_threshold(param.as_double());
                RCLCPP_INFO(this->get_logger(), "Parameter 'threshold' changed to: %f", param.as_double());
            }
            else
            {
                result.successful = false;
                result.reason = "Invalid type for parameter 'threshold'.";
            }
        }
        else if (param.get_name() == "min_samples")
        {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                this->ransac_ball->set_min_samples(param.as_int());
                RCLCPP_INFO(this->get_logger(), "Parameter 'min_samples' changed to: %ld", param.as_int());
            }
            else
            {
                result.successful = false;
                result.reason = "Invalid type for parameter 'min_samples'.";
            }
        }
    }
    return result;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<RansacBallNode> node = std::make_shared<RansacBallNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}