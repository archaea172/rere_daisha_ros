#include <rere_daisha_ros/sm_rere_daisha.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  smacc2::run<sm_rere_daisha::SmRereDaisha>(); // ステートマシンを実行
  rclcpp::shutdown();
  return 0;
}