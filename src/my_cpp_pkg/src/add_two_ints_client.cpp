#include "rclcpp/rclcpp.hpp"  // ROS 2 の基本ヘッダー
#include "my_cpp_pkg/srv/add_three_ints.hpp"  // カスタムサービス型のヘッダー
#include <chrono>  // 時間処理用（wait用）
#include <cstdlib>  // 標準ライブラリ（数値処理など）
#include <memory>  // std::shared_ptr

using namespace std::chrono_literals;  // 1s などの時間単位を使えるようにする

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);  // ROS 2 通信初期化

  auto node = rclcpp::Node::make_shared("add_three_ints_client");  // ノードを作成（名前：add_three_ints_client）

  // クライアント作成（AddThreeInts型、サービス名："add_three_ints"）
  auto client = node->create_client<my_cpp_pkg::srv::AddThreeInts>("add_three_ints");

  // サーバーが利用可能になるまで待機（1秒ずつ確認）
  while (!client->wait_for_service(0.1s)) {
    if (!rclcpp::ok()) {  // 強制終了された場合
      RCLCPP_ERROR(node->get_logger(), "サービス待機中に中断されました。");
      return 1;  // 異常終了
    }
    RCLCPP_INFO(node->get_logger(), "サービス待機中...");  // サービスが見つかるまでログ出力
  }

  // リクエストオブジェクトを作成し、a、b、cに値を設定
  auto request = std::make_shared<my_cpp_pkg::srv::AddThreeInts::Request>();
  request->a = 42;
  request->b = 58;
  request->c = 20;
  request->d = 50;

  // 非同期でリクエストを送信し、結果（Future）を取得
  auto result_future = client->async_send_request(request);

  // Futureが完了するまで待機（成功したら結果を出力）
  if (rclcpp::spin_until_future_complete(node, result_future) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(node->get_logger(), "結果: %ld + %ld + %ld = %ld", 
                request->a, request->b, request->c, result_future.get()->sum);  // 結果を表示
  } else {
    RCLCPP_ERROR(node->get_logger(), "サービス呼び出しに失敗しました。");  // エラー出力
  }

  rclcpp::shutdown();  // ROS 2 通信終了
  return 0;  // 正常終了
}
