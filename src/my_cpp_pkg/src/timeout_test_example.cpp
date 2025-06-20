#include "serchID.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    auto node = rclcpp::Node::make_shared("timeout_test_node");
    RCLCPP_INFO(node->get_logger(), "タイムアウトテスト開始");

    // デバイス名を設定
    SearchID::DeviceManager::getInstance().setDeviceName("PC1");
    
    // 存在しないデバイスを登録してタイムアウトをテスト
    std::string fake_device_name = "fake_device";
    std::string fake_port = "/dev/ttyUSB99";  // 存在しないポート
    
    RCLCPP_INFO(node->get_logger(), "存在しないデバイス %s (%s) を登録中...", 
               fake_device_name.c_str(), fake_port.c_str());
    
    // デバイスを作成（失敗するはず）
    auto fake_device = std::make_shared<SerialCommunication>("fake_serial_node", fake_port, B9600, "\r\n");
    
    // デバイスを強制的に登録（シリアルポートは開けないがタイムアウトテストのため）
    SearchID::DeviceManager::getInstance().registerDevice(fake_device_name, fake_device);
    
    RCLCPP_INFO(node->get_logger(), "IDリクエストを送信中...");
    
    // IDリクエストを送信（タイムアウトするはず）
    SearchID::DeviceManager::getInstance().sendIDRequestWithTimeout(fake_device_name, "ID", 2000);  // 2秒タイムアウト
    
    RCLCPP_INFO(node->get_logger(), "タイムアウトを待機中...");
    
    // タイマーが動作するようにスピン
    rclcpp::Rate rate(10);  // 10Hz
    auto start_time = std::chrono::steady_clock::now();
    
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed > std::chrono::seconds(5)) {
            RCLCPP_INFO(node->get_logger(), "5秒経過、テスト終了");
            break;
        }
        
        rate.sleep();
    }
    
    // 使用不可デバイスをチェック
    auto unusable_devices = SearchID::DeviceManager::getInstance().getUnusableDevices();
    if (unusable_devices.find(fake_device_name) != unusable_devices.end()) {
        RCLCPP_INFO(node->get_logger(), "SUCCESS: デバイス %s は使用不可としてマークされました", 
                   fake_device_name.c_str());
        std::cout << "SUCCESS: Device " << fake_device_name << " marked as unusable" << std::endl;
    } else {
        RCLCPP_ERROR(node->get_logger(), "FAILED: デバイス %s は使用不可としてマークされませんでした", 
                    fake_device_name.c_str());
        std::cout << "FAILED: Device " << fake_device_name << " not marked as unusable" << std::endl;
    }
    
    RCLCPP_INFO(node->get_logger(), "タイムアウトテスト完了");
    rclcpp::shutdown();
    return 0;
}
