#include "integration.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <thread>
#include <chrono>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto system = create_integrated_system("PC1", true, 115200);

    // バックグラウンドでID探索を実行
    std::thread init_thread([&]() {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        RCLCPP_INFO(system->get_logger(), "=== マイコンID探索を開始 ===");
        system->discoverMicrocontrollerIDs(2000); 
        RCLCPP_INFO(system->get_logger(), "=== ID探索完了。待機モードです。 ===");
    });

    std::cout << "\nPC1 高速非同期サーバー 起動。PC2からのリクエストを待機中..." << std::endl;

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4); // 4スレッドで並列処理
    executor.add_node(system);
    for(const auto& dev : system->getSerialDevices()) {
        executor.add_node(dev);
    }
    executor.spin();
    
    if (init_thread.joinable()) init_thread.join();
    rclcpp::shutdown();
    return 0;
}