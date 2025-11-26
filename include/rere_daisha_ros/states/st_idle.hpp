#pragma once
#include <smacc2/smacc.hpp>
#include "../sm_rere_daisha.hpp"
using namespace sm_rere_daisha::orthogonals;
using namespace cl_ros_timer;

namespace sm_rere_daisha {
namespace states {

// 次のStateの先行宣言
struct StActive;

struct StIdle : smacc2::SmaccState<StIdle, sm_rere_daisha::SmRereDaisha> {
  using SmaccState::SmaccState;

  // --- 遷移マップ (Transition Map) ---
  typedef mpl::list<
    // EvTimerイベントが発生したら、StActiveへ遷移(Transition)する
    Transition<EvTopicMessage<ClKeyboard, OrKeyboard>, StActive>
  > reactions;

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