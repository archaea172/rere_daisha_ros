#include "ransac_ball_partly.hpp"

RansacBallPartlyNode::RansacBallPartlyNode()
: rclcpp_lifecycle::LifecycleNode(std::string("ransac_ball_partly_node"))
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
        std::bind(&RansacBallPartlyNode::parameters_callback, this, _1)
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
        std::string("/ldlidar_node/scan"),
        rclcpp::SystemDefaultsQoS(),
        std::bind(&RansacBallPartlyNode::lidar_callback, this, _1)
    );

    this->ball_yolo_subscriber = this->create_subscription<rere_daisha_msgs::msg::BallPositionArray>(
        std::string("ball_position_yolo"),
        rclcpp::SystemDefaultsQoS(),
        std::bind(&RansacBallPartlyNode::ball_callback, this, _1)
    );

    this->ransac_timer = this->create_wall_timer(
        0.01s,
        std::bind(&RansacBallPartlyNode::ransac_timer_callback, this)
    );
    /*node func end*/

    return CallbackReturn::SUCCESS;
}

RansacBallPartlyNode::CallbackReturn RansacBallPartlyNode::on_deactivate(const rclcpp_lifecycle::State &state)
{
    this->nearest_ball_position->on_deactivate();
    this->lidar_subscriber.reset();
    this->ball_yolo_subscriber.reset();
    this->ransac_timer.reset();
    return CallbackReturn::SUCCESS;
}

RansacBallPartlyNode::CallbackReturn RansacBallPartlyNode::on_cleanup(const rclcpp_lifecycle::State &state)
{
    this->lidar_subscriber.reset();
    this->ball_yolo_subscriber.reset();
    this->ransac_timer.reset();
    return CallbackReturn::SUCCESS;
}

RansacBallPartlyNode::CallbackReturn RansacBallPartlyNode::on_error(const rclcpp_lifecycle::State &state)
{
    return CallbackReturn::SUCCESS;
}

RansacBallPartlyNode::CallbackReturn RansacBallPartlyNode::on_shutdown(const rclcpp_lifecycle::State &state)
{
    this->lidar_subscriber.reset();
    this->ball_yolo_subscriber.reset();
    this->ransac_timer.reset();
    return CallbackReturn::SUCCESS;
}

void RansacBallPartlyNode::lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr rxdata)
{
    this->scan_ = *rxdata;
}

void RansacBallPartlyNode::ball_callback(const rere_daisha_msgs::msg::BallPositionArray::SharedPtr rxdata)
{
    this->ball_position_array_ = *rxdata;
}

void RansacBallPartlyNode::ransac_timer_callback()
{
    if (this->scan_.header.stamp.sec == 0) return;
    else if (this->ball_position_array_.balls.empty()) return;

    float nearest_length = 0;
    size_t nearest_index;
    
    for (size_t i = 0; i < this->ball_position_array_.balls.size(); i++)
    {
        float ball_x = this->ball_position_array_.balls[i].position.x;
        float ball_y = this->ball_position_array_.balls[i].position.y;
        
        float length = std::hypot(ball_x, ball_y);
        if (nearest_length == 0)
        {
            nearest_length = length;
            nearest_index = i;
        }
        else if (nearest_length > length)
        {
            nearest_length = length;
            nearest_index = i;
        }
    }

    float nearest_ball_x = this->ball_position_array_.balls[nearest_index].position.x;
    float nearest_ball_y = this->ball_position_array_.balls[nearest_index].position.y;

    float nearest_ball_rad = std::atan2(nearest_ball_y, nearest_ball_x);
    
    while (nearest_ball_rad > 2*M_PI) nearest_ball_rad -= 2*M_PI;
    while (nearest_ball_rad < 0) nearest_ball_rad += 2*M_PI;

    float nearest_ball_field_rad = atan(this->ball_r/nearest_length);

    float min_angle_limit = nearest_ball_rad - nearest_ball_field_rad;
    float max_angle_limit = nearest_ball_rad + nearest_ball_field_rad;
    
    std::vector<std::vector<float>> nearest_ball_points;
    float angle = this->scan_.angle_min;
    for (size_t i = 0; i < this->scan_.ranges.size(); i++)
    {
        if (angle > nearest_ball_rad - nearest_ball_field_rad)
        {
            float point_x = this->scan_.ranges[i]*std::cos(angle);
            float point_y = this->scan_.ranges[i]*std::sin(angle);
            std::vector<float> point = {point_x, point_y};
            nearest_ball_points.push_back(point);
        }
        else if (angle < nearest_ball_rad - nearest_ball_field_rad) break;

        angle += this->scan_.angle_increment;
    }

    rere_daisha_msgs::msg::BallPosition txdata;
    txdata.class_id = this->ball_position_array_.balls[nearest_index].class_id;

    if (nearest_ball_points.empty()) return;

    std::vector<std::vector<float>> ball_centers = this->ransac_ball->run(nearest_ball_points);
    if (ball_centers.empty())
    {
        txdata.position.x = nearest_ball_points[0][0];
        txdata.position.y = nearest_ball_points[0][1];
    }
    else
    {
        txdata.position.x = ball_centers[0][0];
        txdata.position.y = ball_centers[0][1];
    }

    if (this->nearest_ball_position->is_activated())
    {
        this->nearest_ball_position->publish(txdata);
        
    }
}

rcl_interfaces::msg::SetParametersResult RansacBallPartlyNode::parameters_callback(
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
    std::shared_ptr<RansacBallPartlyNode> node = std::make_shared<RansacBallPartlyNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}