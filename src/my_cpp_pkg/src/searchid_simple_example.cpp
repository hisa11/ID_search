#include "serial.hpp"
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
    std::unordered_map<std::string, std::string> device_id_map; // ID -> device index
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "Scanning for serial devices...");
    
    // シリアルデバイスを接続
    for (const auto& port : ports) {
        RCLCPP_INFO(rclcpp::get_logger("main"), "Trying to connect to %s", port.c_str());
        
        std::string node_name = "device" + std::to_string(connected_devices.size()) + "_node";
        auto device = std::make_shared<SerialCommunication>(node_name, port, B9600, "|");
            
        if (device->initialize()) {
            RCLCPP_INFO(rclcpp::get_logger("main"), "Successfully connected to %s", port.c_str());
            connected_devices.push_back(device);
        } else {
            RCLCPP_WARN(rclcpp::get_logger("main"), "Failed to connect to %s", port.c_str());
        }
    }

    if (connected_devices.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "No devices connected");
        return 1;
    }

    RCLCPP_INFO(rclcpp::get_logger("main"), "Connected %zu device(s)", connected_devices.size());

    // 各デバイスに受信コールバックを設定してID探索
    for (size_t i = 0; i < connected_devices.size(); ++i) {
        auto& device = connected_devices[i];
        
        // 受信コールバックを設定
        device->setReceiveCallback([&device_id_map, i](const std::string& message) {
            RCLCPP_INFO(rclcpp::get_logger("main"), "Device %zu received: '%s'", i, message.c_str());
            
            // ID応答かチェック（101,device_id,の形式）
            if (message.find("101,") == 0) {
                // 最初のコンマの後から2番目のコンマまでを抽出
                size_t first_comma = message.find(',');
                if (first_comma != std::string::npos) {
                    size_t second_comma = message.find(',', first_comma + 1);
                    if (second_comma != std::string::npos) {
                        std::string device_id = message.substr(first_comma + 1, second_comma - first_comma - 1);
                        device_id_map[device_id] = std::to_string(i);
                        RCLCPP_INFO(rclcpp::get_logger("main"), "Device %zu has ID: %s", i, device_id.c_str());
                    } else {
                        // 2番目のコンマがない場合、最後まで取得
                        std::string device_id = message.substr(first_comma + 1);
                        device_id_map[device_id] = std::to_string(i);
                        RCLCPP_INFO(rclcpp::get_logger("main"), "Device %zu has ID: %s", i, device_id.c_str());
                    }
                }
            }
        });
        
        // 受信開始
        device->startReceiving();
    }
    
    // ROSのエグゼキューターを使ってスピン（受信処理のため）
    rclcpp::executors::MultiThreadedExecutor executor;
    for (auto& device : connected_devices) {
        executor.add_node(device);
    }
    
    // 全デバイスに並列でID探索メッセージを送信（高速化）
    for (size_t i = 0; i < connected_devices.size(); ++i) {
        std::string id_message = "1,PC1,0,|";
        connected_devices[i]->sendRaw(id_message);
        RCLCPP_INFO(rclcpp::get_logger("main"), "Sent ID request to device %zu", i);
    }
    
    // ID応答を待機
    RCLCPP_INFO(rclcpp::get_logger("main"), "Waiting for ID responses...");
    
    // 1秒間待機してID応答を収集（さらに高速化）
    // 全デバイスから応答があれば早期終了
    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(1)) {
        executor.spin_some(std::chrono::milliseconds(10));  // より高頻度でチェック
        
        // 全デバイスから応答があったかチェック
        if (device_id_map.size() >= connected_devices.size()) {
            RCLCPP_INFO(rclcpp::get_logger("main"), "All devices responded, proceeding early");
            break;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(2));  // 待機時間を短縮
    }

    // IDに対応したメッセージを送信
    RCLCPP_INFO(rclcpp::get_logger("main"), "Sending messages based on IDs...");
    
    for (const auto& id_pair : device_id_map) {
        std::string device_id = id_pair.first;
        int device_index = std::stoi(id_pair.second);
        
        if (device_index >= 0 && device_index < static_cast<int>(connected_devices.size())) {
            std::string message;
            if (device_id == "nucleo1") {
                message = "asagohan|";
            } else if (device_id == "nucleo2") {
                message = "bangohan|";
            } else {
                message = "hello_" + device_id + "|";
            }
            
            connected_devices[device_index]->sendRaw(message);
            RCLCPP_INFO(rclcpp::get_logger("main"), "Sent '%s' to device %d (ID: %s)", 
                       message.c_str(), device_index, device_id.c_str());
        }
    }

    // 送信完了（最高速化）
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "Communication completed. Shutting down...");
    RCLCPP_INFO(rclcpp::get_logger("main"), "Program finished successfully");
    
    _exit(0);
}
