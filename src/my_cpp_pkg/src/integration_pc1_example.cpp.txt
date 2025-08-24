#include "integration.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <thread>
#include <chrono>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto system = create_integrated_system("PC1", true, 115200);

    std::cout << "\nPC1 高速非同期サーバー 起動。PC2からのリクエストを待機中..." << std::endl;

    // エグゼキューターのセットアップ
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(system);
    for(const auto& dev : system->getSerialDevices()) {
        executor.add_node(dev);
    }

    // バックグラウンドでID探索を実行（detachして独立実行）
    std::thread init_thread([&]() {
        // std::this_thread::sleep_for(std::chrono::seconds(1)); // エグゼキューター起動を待つ
        RCLCPP_INFO(system->get_logger(), "=== マイコンID探索を開始 ===");
        auto discovered = system->discoverMicrocontrollerIDs(3000);
        RCLCPP_INFO(system->get_logger(), "=== ID探索完了。発見されたマイコン: %zu個 ===", discovered.size());
        
        if (!discovered.empty()) {
            RCLCPP_INFO(system->get_logger(), "発見されたマイコンID:");
            for (const auto& mc_id : discovered) {
                RCLCPP_INFO(system->get_logger(), "  - %s", mc_id.c_str());
            }
        } else {
            RCLCPP_WARN(system->get_logger(), "マイコンが発見されませんでした。マイコンのプログラムが正しく動作しているか確認してください。");
        }
        RCLCPP_INFO(system->get_logger(), "=== 待機モードです。マイコン間通信とPC間通信の準備完了 ===");
    });
    init_thread.detach(); // スレッドを独立実行

    // メインループ（Ctrl+Cで終了）
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}