#include "serchID.hpp"
#include <iostream>
#include <rclcpp/executors.hpp>
#include <thread>
#include <chrono>
#include <future>
#include <signal.h>

// グローバル変数（シグナルハンドラー用）
volatile std::sig_atomic_t g_signal_received = 0;

void signal_handler(int signal) {
    g_signal_received = signal;
}

int main(int argc, char *argv[])
{
    // シグナルハンドラーを設定（安全な終了のため）
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    
    rclcpp::init(argc, argv);

    // デバイス名を設定（PC名として使用）
    SearchID::setDeviceName("PC1");

    // デバイスを登録
    auto device1 = SearchID::createAndRegisterDevice(
        "arduino1", "arduino1_node", "/dev/ttyACM0", B9600, "|");

    auto device2 = SearchID::createAndRegisterDevice(
        "arduino2", "arduino2_node", "/dev/ttyACM1", B9600, "|");

    auto device3 = SearchID::createAndRegisterDevice(
        "sensor", "sensor_node", "/dev/ttyACM2", B9600, "|");

    // 初期化されたデバイス数をカウント
    int initialized_count = 0;
    if (device1) initialized_count++;
    if (device2) initialized_count++;
    if (device3) initialized_count++;

    if (initialized_count == 0) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "No devices initialized. Exiting.");
        return -1;
    }

    RCLCPP_INFO(rclcpp::get_logger("main"), "Initialized %d device(s)", initialized_count);

    // ダミーノードを作成（MultiThreadedExecutorで必要）
    auto node = rclcpp::Node::make_shared("id_send_example");

    RCLCPP_INFO(rclcpp::get_logger("main"), "ID Send Example started");

    // MultiThreadedExecutorを使用して複数のノードをスピン
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    // 初期化されたデバイスのノードも追加
    if (device1) executor.add_node(device1);
    if (device2) executor.add_node(device2);
    if (device3) executor.add_node(device3);
    
    // バックグラウンドでエグゼキューターを開始
    std::thread executor_thread([&executor]() {
        executor.spin();
    });
    
    // エグゼキューター開始の少し待機（5ms）
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    RCLCPP_INFO(rclcpp::get_logger("main"), "=== Starting Device Discovery ===");
    
    // デバイス探索を開始（超短いタイムアウト：100ms）
    SearchID::start(100);
    
    // 最適化されたタイトループでnucleo1を待機・送信
    bool sent_asagohan = false;
    auto start_time = std::chrono::steady_clock::now();
    
    while (!sent_asagohan) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();
        
        // 最大1秒でタイムアウト
        if (elapsed > 1000) {
            RCLCPP_WARN(rclcpp::get_logger("main"), "Timeout: nucleo1 not found after 1 second");
            break;
        }
        
        // IDマップをチェック（10msごと）
        auto id_map = SearchID::getDeviceIDMap();
        
        // nucleo1が見つかった瞬間にasagohanを送信
        if (id_map.find("nucleo1") != id_map.end()) {
            if (SearchID::sendByID("nucleo1", "asagohan")) {
                auto total_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - start_time).count();
                RCLCPP_INFO(rclcpp::get_logger("main"), 
                           "SUCCESS: Sent 'asagohan' to nucleo1 in %ld ms from start", total_time);
                sent_asagohan = true;
            } else {
                RCLCPP_WARN(rclcpp::get_logger("main"), "Failed to send to nucleo1");
            }
        }
        else if (id_map.find("nucleo2") != id_map.end()) {
            if (SearchID::sendByID("nucleo2", "bangohan")) {
                auto total_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - start_time).count();
                RCLCPP_INFO(rclcpp::get_logger("main"), 
                           "SUCCESS: Sent 'bangohan' to nucleo2 in %ld ms from start", total_time);
                sent_asagohan = true;
            } else {
                RCLCPP_WARN(rclcpp::get_logger("main"), "Failed to send to nucleo2");
            }
        }
        
        // 非常に短い待機時間（10ms）
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    // 全デバイスの確認とテストメッセージ送信
    auto final_id_map = SearchID::getDeviceIDMap();
    if (!final_id_map.empty()) {
        RCLCPP_INFO(rclcpp::get_logger("main"), "=== Found Devices ===");
        for (const auto& pair : final_id_map) {
            RCLCPP_INFO(rclcpp::get_logger("main"), 
                       "ID: %s -> Device: %s", pair.first.c_str(), pair.second.c_str());
            
            // nucleo1以外のデバイスにテストメッセージ送信
            if (pair.first != "nucleo1") {
                std::string test_msg = "hello_" + pair.first;
                if (SearchID::sendByID(pair.first, test_msg)) {
                    RCLCPP_INFO(rclcpp::get_logger("main"), 
                               "Sent '%s' to %s", test_msg.c_str(), pair.first.c_str());
                }
            }
        }
    }
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "=== Test completed ===");
    
    // 改善されたクリーンアップ処理
    RCLCPP_INFO(rclcpp::get_logger("main"), "=== Starting cleanup ===");
    
    // 1. 全デバイスの受信を停止
    if (device1) device1->stopReceiving();
    if (device2) device2->stopReceiving();
    if (device3) device3->stopReceiving();
    
    // 2. 少し待機してリソースが解放されるのを待つ
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // 3. エグゼキューターを停止（gracefully）
    executor.cancel();
    
    // 4. スレッドの適切な終了処理
    if (executor_thread.joinable()) {
        auto future = std::async(std::launch::async, [&executor_thread]() {
            executor_thread.join();
        });
        
        // 最大500ms待機してからdetach
        if (future.wait_for(std::chrono::milliseconds(500)) == std::future_status::timeout) {
            RCLCPP_WARN(rclcpp::get_logger("main"), "Executor thread join timeout, detaching");
            executor_thread.detach();
        }
    }
    
    // 5. 明示的にデバイスのリソースを解放
    try {
        device1.reset();
        device2.reset();
        device3.reset();
    } catch (const std::exception& e) {
        RCLCPP_WARN(rclcpp::get_logger("main"), "Exception during device reset: %s", e.what());
    }
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "=== Cleanup completed ===");
    
    // 安全なシャットダウン
    rclcpp::shutdown();
    return 0;
}
    