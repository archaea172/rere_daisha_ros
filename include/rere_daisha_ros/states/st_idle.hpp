#pragma once
#include <smacc2/smacc.hpp>
#include "../sm_rere_daisha.hpp"
using namespace sm_rere_daisha::orthogonals;
using namespace cl_ros_timer;

namespace sm_rere_daisha {
namespace states {

struct EvKeyA : sc::event<EvKeyA> {};
// 次のStateの先行宣言
struct StActive;

struct StIdle : smacc2::SmaccState<StIdle, sm_rere_daisha::SmRereDaisha> {
  using SmaccState::SmaccState;

  // --- 遷移マップ (Transition Map) ---
  typedef mpl::list<
    Transition<EvKeyA, StActive>
  > reactions;
  void runtimeConfigure() {
    ClKeyboard* client;
    this->requiresClient(client);if (client != nullptr) {
      client->onMessageReceived(&StIdle::onKeyReceived, this);
    } else {
      RCLCPP_ERROR(getLogger(), "ClKeyboard not found!");
    }
  }

  void onKeyReceived(const std_msgs::msg::UInt16& msg) {
    // 'a' の文字コードは 97
    if (msg.data == 97) {
      RCLCPP_INFO(getLogger(), "Key 'a' pressed! Posting EvKeyA.");
      this->postEvent<EvKeyA>(); // 自作イベントを発火！ -> 遷移マップが反応する
    }
    // 's' の文字コードは 115
    else if (msg.data == 115) {
      RCLCPP_INFO(getLogger(), "Key 's' pressed. Stay in Idle.");
      // StIdleにいる時に's'(停止)を押されても何もしない（あるいはEvKeySを投げても遷移定義がなければ無視される）
    }
    else {
      RCLCPP_INFO(getLogger(), "Ignored key: %d", msg.data);
    }
  }

  // --- 状態に入った時の処理 ---
  void onEntry() {
    RCLCPP_INFO(getLogger(), "waiting for keyboard input...");
  }

  // --- 状態から出る時の処理 ---
  void onExit() {
    RCLCPP_INFO(getLogger(), "EXITING IDLE STATE");
  }
};

}
}