#include "service.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <vector>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    std::vector<std::string> pc1_microcontrollers = {"maikon1"};
    auto node_pc1 = std::make_shared<Service>("PC1", pc1_microcontrollers);

    std::thread process_thread([&]() {
        RCLCPP_INFO(node_pc1->get_logger(), "[Thread] ネットワークの安定と情報交換を待ちます...");
        std::this_thread::sleep_for(std::chrono::seconds(5));

        RCLCPP_INFO(node_pc1->get_logger(), "[Thread] マイコン所持確認のブロードキャストを開始します。");
        node_pc1->broadcast_to_all_nodes(1, "ALL", {}, pc1_microcontrollers, "PC1からの情報交換要求");
        
        std::this_thread::sleep_for(std::chrono::seconds(3));
        RCLCPP_INFO(node_pc1->get_logger(), "[Thread] ルーティングテストを開始します。");

        std::string immediate_target = "PC2";
        std::string final_destination = "maikon2";
        int64_t request_type = 20;
        std::string message = "maikon2で実行してほしいタスク";

        RCLCPP_INFO(node_pc1->get_logger(), "[Thread] %s経由で%s宛のタスクを送信します。", immediate_target.c_str(), final_destination.c_str());
        node_pc1->send_custom_request(immediate_target, request_type, final_destination, {}, {}, message);
    });

    RCLCPP_INFO(node_pc1->get_logger(), "ノード 'PC1' を起動しました。送受信待機中です...");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_pc1);
    executor.spin();

    if (process_thread.joinable()) process_thread.join();
    rclcpp::shutdown();
    return 0;
}