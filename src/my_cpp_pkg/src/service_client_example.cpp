#include "service.hpp"
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <thread>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // ノードを作成
    auto node_pc2 = std::make_shared<Service>("PC3");

    // =============================================================
    // データ送信を別スレッドで行うように変更
    // =============================================================
    std::thread sender_thread([&]() {
        // PC1 が起動するのを少し待つ
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // 送信データの準備
        std::string target_node = "PC1";
        int64_t request_type = 2;
        std::string dest_node = "PC1";
        std::vector<int64_t> values = {30, 10, 20}; // 例の値を設定
        std::string message = "PC2からの初期リクエスト";

        RCLCPP_INFO(node_pc2->get_logger(), "'%s' にデータを送信します。", target_node.c_str());

        // 非同期でデータを送信
        node_pc2->send_custom_request(target_node, request_type, dest_node, values, message);
    });

    // メインスレッドはすぐに spin を開始し、リクエスト待機に入る
    RCLCPP_INFO(node_pc2->get_logger(), "ノード 'PC2' を起動しました。送受信待機中です...");

    // マルチスレッドエグゼキュータで spin を開始
    // これにより、sender_thread での送信処理と、外部からのリクエスト受信を同時に処理できる
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_pc2);
    executor.spin();

    // プログラム終了時にスレッドを合流させる
    if (sender_thread.joinable()) {
        sender_thread.join();
    }
    
    rclcpp::shutdown();
    return 0;
}