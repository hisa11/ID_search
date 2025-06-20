#include "rclcpp/rclcpp.hpp"                        // ROS 2 の基本ヘッダー（ノード、ロガーなど）
#include "my_cpp_pkg/srv/add_three_ints.hpp"        // カスタムサービス型（AddThreeInts）
#include <memory>                                   // std::shared_ptr を使うために必要

using std::placeholders::_1; // bindで使用するプレースホルダー（第1引数用）
using std::placeholders::_2; // bindで使用するプレースホルダー（第2引数用）

// ノードクラス定義（AddThreeIntsServer）
class AddThreeIntsServer : public rclcpp::Node
{
public:
  AddThreeIntsServer() : Node("server1") // コンストラクタでノード名を設定
  {
    // サービスの作成（型：AddThreeInts、名前："add_three_ints"、コールバック関数を登録）
    service_ = this->create_service<my_cpp_pkg::srv::AddThreeInts>(
        "add_three_ints",                                            // サービス名
        std::bind(&AddThreeIntsServer::handle_service, this, _1, _2) // コールバックをbind
    );
  }

private:  
  // サービスのコールバック関数：リクエストを受けてレスポンスを返す
  void handle_service(
      const std::shared_ptr<my_cpp_pkg::srv::AddThreeInts::Request> request, // クライアントからのリクエスト（a, b, c）
      std::shared_ptr<my_cpp_pkg::srv::AddThreeInts::Response> response)     // サーバーからのレスポンス（sum）
  {
    response->sum = request->a + request->b + request->c; // 3つの値の足し算を実行してsumに代入
    RCLCPP_INFO(this->get_logger(), "リクエスト: a = %ld, b = %ld, c = %ld",
                request->a, request->b, request->c);                   // ログ出力（リクエスト値）
    RCLCPP_INFO(this->get_logger(), "応答: sum = %ld", response->sum); // ログ出力（応答値）
  }

  rclcpp::Service<my_cpp_pkg::srv::AddThreeInts>::SharedPtr service_; // サービスオブジェクト
};

// main関数：ノードを初期化してスピン
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);                             // ROS 2 通信初期化
  rclcpp::spin(std::make_shared<AddThreeIntsServer>()); // ノードをスピンしてサービス待機
  rclcpp::shutdown();                                   // 通信終了処理
  return 0;                                             // 正常終了
}
