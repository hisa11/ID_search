#include "integration.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <numeric> // 結果表示のために追加

// ネットワーク構成をきれいに表示するためのヘルパー関数
void print_network_map(rclcpp::Logger logger, const std::map<std::string, std::vector<std::string>>& network_map)
{
    RCLCPP_INFO(logger, "--- ネットワーク構成 ---");
    if (network_map.empty()) {
        RCLCPP_INFO(logger, "  (探索中またはノードが見つかりません)");
    }
    for (const auto& pair : network_map) {
        std::string mc_list_str;
        if (pair.second.empty()) {
            mc_list_str = "(マイコンなし)";
        } else {
            // vectorの要素を ", " 区切りで連結する
            mc_list_str = std::accumulate(
                std::next(pair.second.begin()), pair.second.end(), pair.second[0],
                [](std::string a, std::string b) {
                    return a + ", " + b;
                });
        }
        RCLCPP_INFO(logger, "  - Node[%s] -> MCs: [%s]", pair.first.c_str(), mc_list_str.c_str());
    }
    RCLCPP_INFO(logger, "-----------------------");
}



int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    // PC3の設定:
    // - ノード名: "PC3"
    // - シリアル通信: false (PC3はマイコンに直接接続しない)
    // - ボーレート: 115200 (未使用だが指定は必要)
    // - 自動探索: true (ネットワーク上の他ノードとマイコンを探す)
    auto system = create_integrated_system("PC3", false, 115200, true);

    std::cout << "\nPC3 ネットワーク監視ノード 起動。" << std::endl;
    std::cout << "バックグラウンドで他ノードの探索を開始します..." << std::endl;

    // エグゼキューターのセットアップ
    // PC3はシリアル通信ノードを持たないので、systemノードだけを追加すれば良い
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(system);

    // エグゼキューターを別スレッドで実行開始
    // これにより、メインスレッドは他の処理（結果表示など）を行える
    std::thread executor_thread([&executor]() {
        executor.spin();
    });

    // 自動探索が完了するのを待つための時間
    // (内部の探索タイムアウト設定より少し長く待つ)
    RCLCPP_INFO(system->get_logger(), "ネットワーク探索のため5秒間待機します...");
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    // 探索結果を取得して表示
    RCLCPP_INFO(system->get_logger(), "最終的な探索結果を取得・表示します。");
    auto final_map = system->getNetworkMicrocontrollerMap();
    print_network_map(system->get_logger(), final_map);
    
    RCLCPP_INFO(system->get_logger(), "=== 待機モードです。Ctrl+Cで終了します ===");

    // プログラムが終了しないように、エグゼキュータースレッドの終了を待つ
    executor_thread.join();
    
    rclcpp::shutdown();
    return 0;
}