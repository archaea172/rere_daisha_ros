#pragma once
#include <smacc2/smacc.hpp>
#include <ros_timer_client/cl_ros_timer.hpp> // 標準ライブラリのタイマークライアント

namespace sm_rere_daisha {
namespace orthogonals {

class OrTimer : public smacc2::Orthogonal<OrTimer> {
public:
  void onInitialize() override {
    // ROS 2のタイマークライアントを作成
    // from_seconds を使う必要があります
    auto client = this->createClient<cl_ros_timer::ClRosTimer>(rclcpp::Duration::from_seconds(10.0));
  }
};

}
}