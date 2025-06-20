#include "serchID.hpp"
#include <iostream>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    
    // デバイス名を設定
    SearchID::setDeviceName("PC1");
    
    // USBデバイスを登録
    auto device1 = SearchID::createAndRegisterDevice(
        "arduino1", "arduino1_node", "/dev/ttyACM0", B9600, "\r\n"
    );
    
    auto device2 = SearchID::createAndRegisterDevice(
        "arduino2", "arduino2_node", "/dev/ttyACM1", B9600, "\r\n"
    );
    
    if (!device1 && !device2) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "No devices initialized successfully");
        rclcpp::shutdown();
        return 1;
    }
    
    // 初期化成功したデバイスで受信を開始
    if (device1) {
        SearchID::DeviceManager::getInstance().startDeviceReceiving("arduino1");
        RCLCPP_INFO(rclcpp::get_logger("main"), "Arduino1 receiving started");
    }
    
    if (device2) {
        SearchID::DeviceManager::getInstance().startDeviceReceiving("arduino2");
        RCLCPP_INFO(rclcpp::get_logger("main"), "Arduino2 receiving started");
    }
    
    // ノードを作成してタイマーを設定
    auto node = rclcpp::Node::make_shared("id_request_timeout_example");
    
    // 5秒後にタイムアウト付きIDリクエストを送信
    auto id_timer = node->create_wall_timer(
        std::chrono::seconds(5),
        []() {
            static bool id_sent = false;
            if (!id_sent) {
                RCLCPP_INFO(rclcpp::get_logger("main"), "Sending ID requests with 100ms timeout...");
                
                // 各デバイスにタイムアウト付きIDリクエストを送信
                auto devices = SearchID::getDevices();
                for (const auto& device_name : devices) {
                    if (SearchID::isOpen(device_name)) {
                        // 100msタイムアウトでIDリクエストを送信
                        SearchID::sendIDRequestWithTimeout(device_name, "ID", 100);
                        RCLCPP_INFO(rclcpp::get_logger("main"), 
                                   "ID request with timeout sent to: %s", device_name.c_str());
                    }
                }
                
                id_sent = true;
            }
        }
    );
    
    // 10秒後にデバイス状態をチェック
    auto status_timer = node->create_wall_timer(
        std::chrono::seconds(10),
        []() {
            static bool status_checked = false;
            if (!status_checked) {
                RCLCPP_INFO(rclcpp::get_logger("main"), "Checking device status...");
                
                auto devices = SearchID::getDevices();
                for (const auto& device_name : devices) {
                    bool usable = SearchID::isDeviceUsable(device_name);
                    RCLCPP_INFO(rclcpp::get_logger("main"), 
                               "Device %s: %s", device_name.c_str(), 
                               usable ? "USABLE" : "NOT USABLE");
                    
                    std::cout << "Device " << device_name << ": " 
                             << (usable ? "USABLE" : "NOT USABLE") << std::endl;
                }
                
                status_checked = true;
            }
        }
    );
    
    // 15秒後に再度テスト
    auto retry_timer = node->create_wall_timer(
        std::chrono::seconds(15),
        []() {
            RCLCPP_INFO(rclcpp::get_logger("main"), "Retrying ID requests...");
            
            auto devices = SearchID::getDevices();
            for (const auto& device_name : devices) {
                if (SearchID::isOpen(device_name)) {
                    SearchID::sendIDRequestWithTimeout(device_name, "STATUS", 100);
                }
            }
        }
    );
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "ID Request Timeout Example started");
    RCLCPP_INFO(rclcpp::get_logger("main"), "Will send ID requests with 100ms timeout in 5 seconds");
    RCLCPP_INFO(rclcpp::get_logger("main"), "Expected response format: '101,value1,value2-value3'");
    RCLCPP_INFO(rclcpp::get_logger("main"), "If no response within 100ms, device will be marked as unusable");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
