#pragma once
#include <smacc2/smacc.hpp>
#include <smacc2/client_bases/smacc_subscriber_client.hpp> // 標準のサブスクライバー
#include <std_msgs/msg/u_int16.hpp> // 文字コードを送る用

namespace sm_rere_daisha {
namespace orthogonals {

// クライアントの型定義（長いので別名をつける）
using ClKeyboard = smacc2::client_bases::SmaccSubscriberClient<std_msgs::msg::UInt16>;

class OrKeyboard : public smacc2::Orthogonal<OrKeyboard> {
public:
  void onInitialize() override {
    // "/keyboard_input" というトピックを監視するクライアントを作成
    auto client = this->createClient<ClKeyboard>("/keyboard_input");
  }
};

} // namespace orthogonals
} // namespace sm_rere_daisha