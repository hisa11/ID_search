#include "service.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // クライアントIDとサービス名を指定してクライアントを作成
    auto client_node = std::make_shared<Client>("add_three_ints_client", "add_three_ints");

    // サービスが利用可能になるまで待機
    if (!client_node->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(client_node->get_logger(), "サービスが利用できません。終了します。");
        rclcpp::shutdown();
        return 1;
    }

    // サービスにリクエスト送信
    int64_t a = 10, b = 20, c = 30;
    if (client_node->send_request(a, b, c)) {
        RCLCPP_INFO(client_node->get_logger(), "レスポンス: %ld", client_node->get_last_result());
    } else {
        RCLCPP_ERROR(client_node->get_logger(), "サービス呼び出しに失敗しました");
    }

    rclcpp::shutdown();
    return 0;
}
