#pragma once
#include <smacc2/smacc.hpp>
#include "../sm_rere_daisha.hpp"

// 名前空間の省略
using namespace sm_rere_daisha::orthogonals;
using namespace cl_ros_timer;

namespace sm_rere_daisha {
namespace states {

// 戻り先の先行宣言
struct StIdle;

struct StActive : smacc2::SmaccState<StActive, SmRereDaisha> {
  using SmaccState::SmaccState;

  // --- 遷移マップ ---
  typedef mpl::list<
    // タイマーイベントで StIdle (アイドル) に戻る
    Transition<EvTimer<ClRosTimer, OrTimer>, StIdle>
  > reactions;

  // --- onEntry ---
  void onEntry() {
    RCLCPP_INFO(getLogger(), "ENTERING ACTIVE STATE");
  }

  // --- onExit ---
  void onExit() {
    RCLCPP_INFO(getLogger(), "EXITING ACTIVE STATE");
  }
};

} // namespace states
} // namespace sm_rere_daisha