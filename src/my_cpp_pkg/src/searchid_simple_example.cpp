#include "serial.hpp"
#include "serchID.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // シリアル通信を宣言 - 利用可能な全ポートを試行
    std::vector<std::string> ports = {"/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2"};
    std::vector<std::shared_ptr<SerialCommunication>> connected_devices;
    
    // SerialCommunicationの静的メソッドを使用してデバイススキャン
    connected_devices = SerialCommunication::scanAndConnectDevices(ports, B9600, "|");

    if (connected_devices.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "No devices connected");
        return 1;
    }

    // SearchID::DeviceManagerを使用してデバイスを一括登録
    SearchID::DeviceManager& manager = SearchID::DeviceManager::getInstance();
    manager.registerDevices(connected_devices);
    
    // PC名を設定
    manager.setDeviceName("PC1");
    
    // ROSのエグゼキューターを使ってスピン（受信処理のため）
    rclcpp::executors::MultiThreadedExecutor executor;
    for (auto& device : connected_devices) {
        executor.add_node(device);
    }
    
    // DeviceManagerのstartメソッドを使用してID探索を実行
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting device ID discovery...");
    manager.start(1000);  // 1秒のタイムアウト
    
    // ID応答を待機
    RCLCPP_INFO(rclcpp::get_logger("main"), "Waiting for ID responses...");
    
    // 1秒間待機してID応答を収集
    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(1)) {
        executor.spin_some(std::chrono::milliseconds(10));
        
        // 全デバイスから応答があったかチェック
        auto device_id_map = manager.getDeviceIDMap();
        if (device_id_map.size() >= connected_devices.size()) {
            RCLCPP_INFO(rclcpp::get_logger("main"), "All devices responded, proceeding early");
            break;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    // IDに対応したメッセージを送信
    RCLCPP_INFO(rclcpp::get_logger("main"), "Sending messages based on IDs...");
    
    auto device_id_map = manager.getDeviceIDMap();
    for (const auto& id_pair : device_id_map) {
        std::string device_id = id_pair.first;
        
        std::string message;
        if (device_id == "nucleo1") {
            message = "asagohan";
        } else if (device_id == "nucleo2") {
            message = "bangohan";
        } else {
            message = "hello_" + device_id;
        }
        
        if (manager.sendByID(device_id, message)) {
            RCLCPP_INFO(rclcpp::get_logger("main"), "Sent '%s' to device ID: %s", 
                       message.c_str(), device_id.c_str());
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("main"), "Failed to send to device ID: %s", 
                        device_id.c_str());
        }
    }

    // 送信完了（最高速化）
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "Communication completed. Shutting down...");
    RCLCPP_INFO(rclcpp::get_logger("main"), "Program finished successfully");
    
    _exit(0);
}
