#include "service.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <vector>

// グローバル変数として受信データを保持
static std::vector<int64_t> received_values_global;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // ノードを作成
    auto node_pc1 = std::make_shared<Service>("PC1");

    // データ受信時に実行するコールバック関数を定義
    auto my_data_handler = [&](const std::shared_ptr<my_cpp_pkg::srv::DataExchange::Request> request) {
        RCLCPP_INFO(node_pc1->get_logger(), "[Main Handler] データを受信しました！ (Type: %ld)", request->request_type);

        if (!request->values.empty()) {
            // 受信した値をグローバル変数に保存
            // 注意：複数スレッドからアクセスする場合はmutex等での保護が必要になる
            received_values_global = request->values;
            RCLCPP_INFO(node_pc1->get_logger(), "[Main Handler] 値 %zu 個を保存しました。", received_values_global.size());
        }
    };

    // 作成したハンドラをノードに登録
    node_pc1->set_data_handler(my_data_handler);

    // データ送信処理を別スレッドで実行
    std::thread sender_thread([&]() {
        // 相手ノード(PC2)が起動するのを待つ
        std::this_thread::sleep_for(std::chrono::seconds(5));
        
        RCLCPP_INFO(node_pc1->get_logger(), "[Sender Thread] 'PC2' に対してデータの送信を開始します。");

        // 送信データの準備
        std::string target_node = "PC2";
        int64_t request_type = 4;
        std::string dest_node = "PC2";
        std::vector<int64_t> values = {11, 22};
        std::string message = "PC1からの定期連絡";
        
        // 非同期でデータを送信
        node_pc1->send_custom_request(target_node, request_type, dest_node, values, message);
    });

    // メインスレッドはエグゼキュータを起動し、コールバック処理（受信）に専念する
    RCLCPP_INFO(node_pc1->get_logger(), "ノード 'PC1' を起動しました。送受信待機中です...");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_pc1);
    executor.spin();

    // プログラム終了時にスレッドを合流
    if (sender_thread.joinable()) {
        sender_thread.join();
    }

    rclcpp::shutdown();
    return 0;
}