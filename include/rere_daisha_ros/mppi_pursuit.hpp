#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <Eigen/Dense>

#include "mppi.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class MPPIPursuitNode : rclcpp_lifecycle::LifecycleNode
{
    public:
        using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
        MPPIPursuitNode();
    private:
        CallbackReturn on_configure(const rclcpp_lifecycle::State &state);
        CallbackReturn on_activate(const rclcpp_lifecycle::State &state);
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state);
        CallbackReturn on_error(const rclcpp_lifecycle::State &state);
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);

        std::unique_ptr<MPPIControler> mppi_controler;

        OnSetParametersCallbackHandle::SharedPtr parameter_callback_hanle_;
        rcl_interfaces::msg::SetParametersResult parameters_callback(
            const std::vector<rclcpp::Parameter> &parameters
        );
        
        int sample_num;
        int dim_num;
        int loop_num;
        double dt;
        Eigen::MatrixXd sig_;
        double max_wheel_vel;
        double wheel_distance;
        Eigen::VectorXd weight_array;
        double lambda_;
};