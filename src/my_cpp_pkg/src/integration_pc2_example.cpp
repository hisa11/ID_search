#include "integration.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <vector>
#include <chrono>
#include <atomic>
#include <future>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto system = create_integrated_system("PC2", false, 0);

    const int NUM_REQUESTS = 100;
    std::atomic<int> response_count{0};
    auto promise = std::make_shared<std::promise<void>>();
    auto future = promise->get_future();

    system->setDataHandler([&](const std::shared_ptr<my_cpp_pkg::srv::DataExchange::Request> request)
                           {
        std::cout << "データハンドラー呼び出し: request_type=" << request->request_type << std::endl;
        if (request->request_type == 102) {
            std::cout << "応答受信! カウント: " << (response_count + 1) << std::endl;
            if (++response_count >= NUM_REQUESTS) {
                try { promise->set_value(); } catch(...) {}
            }
        } });

    std::thread process_thread([&]()
                               {
        std::cout << "\nPC1のサービスを待機中..." << std::endl;
        auto client = system->create_client<my_cpp_pkg::srv::DataExchange>("PC1");
        if (!client->wait_for_service(std::chrono::seconds(5))) {
            std::cerr << "PC1が見つかりませんでした。" << std::endl;
            rclcpp::shutdown();
            return;
        }
        std::cout << "PC1を発見。30ms間隔で" << NUM_REQUESTS << "回の高速送信を開始します。" << std::endl;

        auto start_time = std::chrono::high_resolution_clock::now();

        for (int i = 0; i < NUM_REQUESTS; ++i) {
            std::vector<std::string> data = {"2", "PC2", "nucleo1", "1", "apple" + std::to_string(i)};
            // ★★★ ここで正しく sendToNodeAsync を呼び出す ★★★
             system->sendToNodeAsync("PC1", "nucleo1", data, "高速送信", 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        auto send_end_time = std::chrono::high_resolution_clock::now();
        auto send_duration = std::chrono::duration_cast<std::chrono::milliseconds>(send_end_time - start_time).count();
        
        std::cout << "\n--- 送信完了 ---" << std::endl;
        std::cout << "総時間: " << send_duration << " ms" << std::endl;
        
        std::cout << "\n全ての応答を待っています (タイムアウト5秒)..." << std::endl;
        if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
            auto total_end_time = std::chrono::high_resolution_clock::now();
            auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(total_end_time - start_time).count();
            std::cout << "\n--- テスト結果 ---" << std::endl;
            std::cout << "成功応答回数: " << response_count << "/" << NUM_REQUESTS << std::endl;
            std::cout << "往復総時間: " << total_duration << " ms" << std::endl;
        } else {
            std::cout << "\nタイムアウト。成功応答: " << response_count << "/" << NUM_REQUESTS << std::endl;
        }

        rclcpp::shutdown(); });

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(system);
    executor.spin();

    if (process_thread.joinable())
        process_thread.join();

    return 0;
}