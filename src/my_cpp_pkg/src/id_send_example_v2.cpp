#include "serchID.hpp"
#include <iostream>
#include <rclcpp/executors.hpp>
#include <thread>
#include <chrono>
#include <memory>
#include <atomic>
#include <future>

class SafeIDSendExample {
private:
    std::shared_ptr<SerialCommunication> device1_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::atomic<bool> running_{true};
    std::thread executor_thread_;

public:
    SafeIDSendExample() = default;
    
    ~SafeIDSendExample() {
        cleanup();
    }
    
    bool initialize() {
        try {
            // デバイス名を設定
            SearchID::setDeviceName("PC1");

            // 1つのデバイスのみを使用（安定性のため）
            device1_ = SearchID::createAndRegisterDevice(
                "arduino1", "arduino1_node", "/dev/ttyACM0", B9600, "|");

            if (!device1_) {
                RCLCPP_ERROR(rclcpp::get_logger("SafeIDSend"), "Failed to initialize device");
                return false;
            }

            RCLCPP_INFO(rclcpp::get_logger("SafeIDSend"), "Device initialized successfully");
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("SafeIDSend"), "Exception in initialize: %s", e.what());
            return false;
        }
    }
    
    void run() {
        try {
            // エグゼキューターを開始
            executor_.add_node(device1_);
            
            // バックグラウンドでエグゼキューターを実行
            executor_thread_ = std::thread([this]() {
                while (running_) {
                    executor_.spin_once(std::chrono::milliseconds(10));
                }
            });

            // 少し待機してエグゼキューターが開始されるのを待つ
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            RCLCPP_INFO(rclcpp::get_logger("SafeIDSend"), "=== Starting Device Discovery ===");
            
            // デバイス探索を開始
            SearchID::start(100);
            
            // nucleo1を待機してメッセージ送信
            auto start_time = std::chrono::steady_clock::now();
            bool sent_asagohan = false;
            
            while (!sent_asagohan && running_) {
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - start_time).count();
                
                if (elapsed > 1000) {
                    RCLCPP_WARN(rclcpp::get_logger("SafeIDSend"), "Timeout: nucleo1 not found");
                    break;
                }
                
                auto id_map = SearchID::getDeviceIDMap();
                if (id_map.find("nucleo1") != id_map.end()) {
                    if (SearchID::sendByID("nucleo1", "asagohan")) {
                        auto total_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - start_time).count();
                        RCLCPP_INFO(rclcpp::get_logger("SafeIDSend"), 
                                   "SUCCESS: Sent 'asagohan' to nucleo1 in %ld ms", total_time);
                        sent_asagohan = true;
                    }
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            
            RCLCPP_INFO(rclcpp::get_logger("SafeIDSend"), "=== Test completed ===");
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("SafeIDSend"), "Exception in run: %s", e.what());
        }
    }
    
    void cleanup() {
        try {
            RCLCPP_INFO(rclcpp::get_logger("SafeIDSend"), "=== Starting cleanup ===");
            
            running_ = false;
            
            // デバイスの受信を停止
            if (device1_) {
                device1_->stopReceiving();
            }
            
            // エグゼキューターを停止
            executor_.cancel();
            
            // スレッドの安全な終了（タイムアウト付き）
            if (executor_thread_.joinable()) {
                auto future = std::async(std::launch::async, [this]() {
                    this->executor_thread_.join();
                });
                
                if (future.wait_for(std::chrono::milliseconds(100)) == std::future_status::timeout) {
                    RCLCPP_WARN(rclcpp::get_logger("SafeIDSend"), "Thread join timeout, detaching");
                    executor_thread_.detach();
                } else {
                    // joinが完了した
                }
            }
            
            // デバイスのリセット
            if (device1_) {
                try {
                    executor_.remove_node(device1_);
                } catch (...) {
                    // remove_nodeでエラーが起きても無視
                }
                device1_.reset();
            }
            
            RCLCPP_INFO(rclcpp::get_logger("SafeIDSend"), "=== Cleanup completed ===");
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("SafeIDSend"), "Exception in cleanup: %s", e.what());
        } catch (...) {
            RCLCPP_ERROR(rclcpp::get_logger("SafeIDSend"), "Unknown exception in cleanup");
        }
    }
};

int main(int argc, char *argv[])
{
    try {
        rclcpp::init(argc, argv);
        
        {
            SafeIDSendExample example;
            
            if (example.initialize()) {
                example.run();
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("main"), "Failed to initialize");
                return -1;
            }
        } // SafeIDSendExampleのデストラクターが呼ばれる
        
        rclcpp::shutdown();
        return 0;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception in main: %s", e.what());
        rclcpp::shutdown();
        return -1;
    }
}
